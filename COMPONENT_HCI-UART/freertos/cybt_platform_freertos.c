/***************************************************************************//**
* \file cybt_platform_freertos.c
*
* \brief
* Implementation for BT porting interface on FreeRTOS
*
********************************************************************************
* \copyright
* Copyright 2018-2021 Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "wiced_data_types.h"

#include "cycfg.h"
#include "cyhal_uart.h"
#include "cyhal_gpio.h"
#include "cybt_platform_interface.h"

#include "cyhal_syspm.h"
#include "cybt_platform_task.h"
#include "cybt_platform_trace.h"
#include "cybt_platform_config.h"
#include "cybt_platform_util.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define HCI_SEMAPHORE_MAX_COUNT  (1)
#define HCI_SEMAPHORE_INIT_COUNT (0)

#if( configUSE_TICKLESS_IDLE != 0 )
#define PLATFORM_SLEEP_IDLE_TIMEOUT_MS     (3000)

#define SLEEP_TASK_NAME               "sleep_task"
#define SLEEP_TASK_STACK_SIZE         (1024)
#define SLEEP_TASK_PRIORITY           (CY_RTOS_PRIORITY_REALTIME)
#define SLEEP_TASK_QUEUE_COUNT        (32)
#define SLEEP_TASK_QUEUE_ITEM_SIZE    (sizeof(uint8_t))

#define SLEEP_ACT_START_IDLE_TIMER    (0x20)
#define SLEEP_ACT_STOP_IDLE_TIMER     (0x40)
#define SLEEP_ACT_EXIT_SLEEP_TASK     (0xFF)
#endif

#ifndef XMC7200D_E272K8384
#define BT_POWER_PIN_DIRECTION  CYHAL_GPIO_DIR_OUTPUT
#define BT_POWER_PIN_DRIVE_MODE CYHAL_GPIO_DRIVE_PULLUP
#define BT_POWER_PIN_INIT_VALUE true
#else
#define BT_POWER_PIN_DIRECTION  CYHAL_GPIO_DIR_OUTPUT
#define BT_POWER_PIN_DRIVE_MODE CYHAL_GPIO_DRIVE_STRONG
#define BT_POWER_PIN_INIT_VALUE false
#endif
/*****************************************************************************
 *                           Type Definitions
 *****************************************************************************/
#if( configUSE_TICKLESS_IDLE != 0 )
typedef uint8_t sleep_action_t;
#endif

typedef struct
{
    bool            inited;
    cyhal_uart_t    hal_obj;
    cy_semaphore_t  tx_complete;
    cy_semaphore_t  rx_complete;
    cy_mutex_t      tx_atomic;
    cy_mutex_t      rx_atomic;
} hci_uart_cb_t;


/******************************************************************************
 *                           Variables Definitions
 ******************************************************************************/
hci_uart_cb_t   hci_uart_cb;

#if( configUSE_TICKLESS_IDLE != 0 )
// This timer is only used in active mode,
// hence it's implemented by cy_rtos_timer API .
cy_timer_t      platform_sleep_idle_timer;
bool            platform_sleep_lock = false;

cy_thread_t     sleep_timer_task;
cy_queue_t      sleep_timer_task_queue;
#endif

static cy_timer_t stack_timer;
/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
#if( configUSE_TICKLESS_IDLE != 0 )
void cybt_idle_timer_cback(cy_timer_callback_arg_t arg);

cybt_result_t cybt_send_action_to_sleep_task(sleep_action_t action);
void cybt_sleep_timer_task(cy_thread_arg_t arg);
#endif

#ifdef COMPONENT_55500
static void cybt_enter_autobaud_mode(void);
#endif //COMPONENT_55500
/******************************************************************************
 *                           Function Definitions
 ******************************************************************************/
static void platform_stack_timer_callback(cy_timer_callback_arg_t arg)
{
    cybt_platform_sleep_lock();

    cybt_send_msg_to_hci_rx_task(BT_IND_TO_BTS_TIMER, true);

    cybt_platform_sleep_unlock();
}


void *cybt_platform_malloc(uint32_t req_size)
{
    return malloc((size_t) req_size);
}

void cybt_platform_free(void *p_mem_block)
{
    free(p_mem_block);
}

void cybt_platform_disable_irq(void)
{
    __disable_irq();
}

void cybt_platform_enable_irq(void)
{
    __enable_irq();
}

void cybt_platform_init(void)
{
#if( configUSE_TICKLESS_IDLE != 0 )
    cy_rtos_init_timer(&platform_sleep_idle_timer,
                       CY_TIMER_TYPE_ONCE,
                       cybt_idle_timer_cback,
                       0
                      );
    MAIN_TRACE_DEBUG("cybt_platform_init(): platform_sleep_idle_timer = 0x%x", &platform_sleep_idle_timer);
#endif

    memset(&hci_uart_cb, 0, sizeof(hci_uart_cb_t));

    cy_rtos_init_timer(&stack_timer, CY_TIMER_TYPE_ONCE, platform_stack_timer_callback, (cy_timer_callback_arg_t)NULL);

#if( configUSE_TICKLESS_IDLE != 0 )
    cy_rtos_init_queue(&sleep_timer_task_queue,
                       SLEEP_TASK_QUEUE_COUNT,
                       SLEEP_TASK_QUEUE_ITEM_SIZE
                      );

    cy_rtos_create_thread(&sleep_timer_task,
                          cybt_sleep_timer_task,
                          SLEEP_TASK_NAME,
                          NULL,
                          SLEEP_TASK_STACK_SIZE,
                          SLEEP_TASK_PRIORITY,
                          (cy_thread_arg_t) 0
                         );
#endif
}

void cybt_platform_deinit(void)
{
    MAIN_TRACE_DEBUG("cybt_platform_deinit()");

#if( configUSE_TICKLESS_IDLE != 0 )
    cybt_send_action_to_sleep_task(SLEEP_ACT_EXIT_SLEEP_TASK);
#endif

    cy_rtos_deinit_timer(&stack_timer);

#if( configUSE_TICKLESS_IDLE != 0 )
    cy_rtos_deinit_timer(&platform_sleep_idle_timer);
#endif
}

void cybt_platform_sleep_lock(void)
{
#if( configUSE_TICKLESS_IDLE != 0 )
    cybt_platform_disable_irq();

    if(false == platform_sleep_lock)
    {
        cyhal_syspm_lock_deepsleep();

        platform_sleep_lock = true;
    }

    cybt_send_action_to_sleep_task(SLEEP_ACT_STOP_IDLE_TIMER);
    cybt_platform_enable_irq();
#endif
}

void cybt_platform_sleep_unlock(void)
{
#if( configUSE_TICKLESS_IDLE != 0 )
    cybt_platform_disable_irq();

    if(true == platform_sleep_lock)
    {
        cybt_send_action_to_sleep_task(SLEEP_ACT_START_IDLE_TIMER);
        cybt_platform_enable_irq();
    }
    else
    {
        cybt_platform_enable_irq();
    }
#endif
}

uint64_t cybt_platform_get_tick_count_us(void)
{
    static cy_time_t last_time_in_ms = 0;
    static uint64_t abs_time_cnt_in_us_hi = 0;
    cy_time_t cur_time_in_ms;
    uint64_t cur_time_in_ms64 = 0;
    uint64_t cur_time_in_us64 = 0;

    cy_rtos_get_time(&cur_time_in_ms);

    if(cur_time_in_ms < last_time_in_ms)
    {
        abs_time_cnt_in_us_hi += 0x100000000;
    }

    last_time_in_ms = cur_time_in_ms;

    cur_time_in_ms64 = cur_time_in_ms + abs_time_cnt_in_us_hi;
    cur_time_in_us64 = (cur_time_in_ms64 * 1000);
    return (cur_time_in_us64);
}

void cybt_platform_set_next_timeout(uint64_t abs_tick_us_to_expire)
{
    uint64_t curr_time_in_us = cybt_platform_get_tick_count_us();
    uint64_t time_to_expire_in_us = abs_tick_us_to_expire - curr_time_in_us;

    if(abs_tick_us_to_expire <= curr_time_in_us)
    {
        // Already expired...
        cybt_send_msg_to_hci_rx_task(BT_IND_TO_BTS_TIMER, true);

        return;
    }

    {
        cy_rslt_t result;
        cy_time_t next_timeout = (cy_time_t)(time_to_expire_in_us/1000);

        /* No need to stop this timer, internally FREE-RTOS restarting the timer
         * Leaving reminder (~1ms), Its ok verified */
        result = cy_rtos_start_timer(&stack_timer, next_timeout);
        if(CY_RSLT_SUCCESS != result)
        {
            MAIN_TRACE_DEBUG("timer failed to start %u\n", next_timeout);
        }
    }
}

__attribute__((weak)) cybt_result_t cybt_debug_uart_send_trace(uint16_t length, uint8_t* p_data)
{
	return CYBT_SUCCESS;
}

void cybt_platform_log_print(const char *fmt_str, ...)
{
    char buffer[CYBT_TRACE_BUFFER_SIZE];
    va_list ap;
    int len;

    va_start(ap, fmt_str);
    len = vsnprintf(buffer, CYBT_TRACE_BUFFER_SIZE, fmt_str, ap);
    va_end(ap);

    cybt_debug_uart_send_trace(len, (uint8_t*)buffer);
}

static void cybt_uart_rx_not_empty(void)
{
    cyhal_uart_enable_event(&hci_uart_cb.hal_obj,
                            CYHAL_UART_IRQ_RX_NOT_EMPTY,
                            CYHAL_ISR_PRIORITY_DEFAULT,
                            false
                           );

    cybt_send_msg_to_hci_rx_task(BT_IND_TO_HCI_DATA_READY_UNKNOWN, true);
}

uint32_t uart_tx_done_cnt = 0;
static void cybt_uart_tx_done_irq(void)
{
    cy_rtos_set_semaphore(&hci_uart_cb.tx_complete, true);
}

static void cybt_uart_rx_done_irq(void)
{
    cy_rtos_set_semaphore(&hci_uart_cb.rx_complete, true);
}

static void cybt_uart_irq_handler(void *handler_arg, cyhal_uart_event_t event)
{
    switch(event)
    {
        case CYHAL_UART_IRQ_RX_NOT_EMPTY:
            cybt_uart_rx_not_empty();
            break;
        case CYHAL_UART_IRQ_TX_DONE:
            cybt_uart_tx_done_irq();
            break;
        case CYHAL_UART_IRQ_RX_DONE:
            cybt_uart_rx_done_irq();
            break;
        default:
            break;
    }
}

void cybt_platform_assert_bt_wake(void)
{
    if(true == cybt_platform_get_sleep_mode_status())
    {
        bool wake_polarity;
        const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();

        switch(p_bt_platform_cfg->controller_config.sleep_mode.device_wake_polarity)
        {
            case CYBT_WAKE_ACTIVE_LOW:
                wake_polarity = false;
                break;
            case CYBT_WAKE_ACTIVE_HIGH:
                wake_polarity = true;
                break;
            default:
                HCIDRV_TRACE_ERROR("ASSERT_BT_WAKE: unknown polarity (%d)",
                                   p_bt_platform_cfg->controller_config.sleep_mode.device_wake_polarity
                                  );
                return;
        }

        cyhal_gpio_write(p_bt_platform_cfg->controller_config.sleep_mode.device_wakeup_pin,
                         wake_polarity
                        );
    }
}

void cybt_platform_deassert_bt_wake(void)
{
    if(true == cybt_platform_get_sleep_mode_status())
    {
        bool sleep_polarity;
        const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();

        switch(p_bt_platform_cfg->controller_config.sleep_mode.device_wake_polarity)
        {
            case CYBT_WAKE_ACTIVE_LOW:
                sleep_polarity = true;
                break;
            case CYBT_WAKE_ACTIVE_HIGH:
                sleep_polarity = false;
                break;
            default:
                HCIDRV_TRACE_ERROR("DEASSERT_BT_WAKE: unknown polarity (%d)",
                                   p_bt_platform_cfg->controller_config.sleep_mode.device_wake_polarity
                                  );
                return;
        }

        cyhal_gpio_write(p_bt_platform_cfg->controller_config.sleep_mode.device_wakeup_pin,
                         sleep_polarity
                        );
    }
}


void cybt_host_wake_irq_handler(void *callback_arg, cyhal_gpio_event_t event)
{
    const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();

    switch(event)
    {
        case CYHAL_GPIO_IRQ_RISE:
            if(CYBT_WAKE_ACTIVE_HIGH == p_bt_platform_cfg->controller_config.sleep_mode.host_wake_polarity)
            {
                cybt_platform_sleep_lock();
            }
            else
            {
                cybt_platform_sleep_unlock();
            }
            break;
        case CYHAL_GPIO_IRQ_FALL:
            if(CYBT_WAKE_ACTIVE_LOW == p_bt_platform_cfg->controller_config.sleep_mode.host_wake_polarity)
            {
                cybt_platform_sleep_lock();
            }
            else
            {
                cybt_platform_sleep_unlock();
            }
            break;
        default:
            break;
    }
}

cybt_result_t cybt_platform_hci_open(void *p_arg)
{
    cyhal_uart_event_t enable_irq_event = (cyhal_uart_event_t)(CYHAL_UART_IRQ_RX_DONE
                                           | CYHAL_UART_IRQ_TX_DONE
                                           | CYHAL_UART_IRQ_RX_NOT_EMPTY
                                          );
    uint32_t actual_baud_rate;
    cy_rslt_t result;
    cyhal_uart_cfg_t bt_uart_cfg = {0};
    UNUSED_VARIABLE(p_arg);
    const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();

    #ifdef COMPONENT_55500
    uint32_t platform_baud_download;
    #endif

    if(true == hci_uart_cb.inited)
    {
        return  CYBT_SUCCESS;
    }

    memset(&hci_uart_cb, 0, sizeof(hci_uart_cb_t));

    cy_rtos_init_semaphore(&hci_uart_cb.tx_complete,
                           HCI_SEMAPHORE_MAX_COUNT,
                           HCI_SEMAPHORE_INIT_COUNT
                          );
    cy_rtos_init_semaphore(&hci_uart_cb.rx_complete,
                           HCI_SEMAPHORE_MAX_COUNT,
                           HCI_SEMAPHORE_INIT_COUNT
                          );
    cy_rtos_init_mutex(&hci_uart_cb.tx_atomic);
    cy_rtos_init_mutex(&hci_uart_cb.rx_atomic);

#if( configUSE_TICKLESS_IDLE != 0 )
    if((CYBT_SLEEP_MODE_ENABLED == p_bt_platform_cfg->controller_config.sleep_mode.sleep_mode_enabled)
      && (NC != p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin))
    {
        result = cyhal_gpio_init(p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin,
                                 CYHAL_GPIO_DIR_INPUT,
                                 CYHAL_GPIO_DRIVE_NONE,
                                 0
                                );
        //If the host_wakeup_pin already initialized by the device configurator or application, we get CYHAL_HWMGR_RSLT_ERR_INUSE.
        if(CYHAL_HWMGR_RSLT_ERR_INUSE == result)
        {
            HCIDRV_TRACE_ERROR("hci_open(): HostWakeup pin already initialized by the app/configurator -  status: (0x%x)", result);
        }
        //For any other error, we return init failure.
        else if(CY_RSLT_SUCCESS != result)
        {
            HCIDRV_TRACE_ERROR("hci_open(): Init HostWakeup pin failed (0x%x)", result);
            return CYBT_ERR_GPIO_HOST_WAKE_INIT_FAILED;
        }
    }

    if((CYBT_SLEEP_MODE_ENABLED == p_bt_platform_cfg->controller_config.sleep_mode.sleep_mode_enabled)
       && (NC != p_bt_platform_cfg->controller_config.sleep_mode.device_wakeup_pin)
       && (NC != p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin)
      )
    {
        if(CYBT_WAKE_ACTIVE_HIGH != p_bt_platform_cfg->controller_config.sleep_mode.host_wake_polarity
           && CYBT_WAKE_ACTIVE_LOW != p_bt_platform_cfg->controller_config.sleep_mode.host_wake_polarity
          )
        {
            HCIDRV_TRACE_ERROR("hci_open(): Unknown host wake polarity(%d)",
                               p_bt_platform_cfg->controller_config.sleep_mode.host_wake_polarity
                              );
            return  CYBT_ERR_GENERIC;
        }

#if (CYHAL_API_VERSION >= 2)
        static cyhal_gpio_callback_data_t cb_data = { .callback = cybt_host_wake_irq_handler, .callback_arg = NULL };
        cyhal_gpio_register_callback(p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin,
                                     &cb_data
                                    );
#else
        cyhal_gpio_register_callback(p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin,
                                     cybt_host_wake_irq_handler,
                                     NULL
                                    );
#endif
        cyhal_gpio_enable_event(p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin,
                                CYHAL_GPIO_IRQ_BOTH,
                                CYHAL_ISR_PRIORITY_DEFAULT,
                                true
                               );
    }
#endif

    if((CYBT_SLEEP_MODE_ENABLED == p_bt_platform_cfg->controller_config.sleep_mode.sleep_mode_enabled)
        && (NC != p_bt_platform_cfg->controller_config.sleep_mode.device_wakeup_pin))
    {
        result = cyhal_gpio_init(p_bt_platform_cfg->controller_config.sleep_mode.device_wakeup_pin,
                                 CYHAL_GPIO_DIR_OUTPUT,
                                 CYHAL_GPIO_DRIVE_STRONG,
                                 0
                                );
        //If the device_wakeup_pin already initialized by the device configurator or application, we get CYHAL_HWMGR_RSLT_ERR_INUSE.
        if(CYHAL_HWMGR_RSLT_ERR_INUSE == result)
        {
            HCIDRV_TRACE_ERROR("hci_open(): DevWakeup pin already initialized by the app/configurator -  status: (0x%x)", result);
        }
        //For any other error, we return init failure.
        else if(CY_RSLT_SUCCESS != result)
        {
            HCIDRV_TRACE_ERROR("hci_open(): Init DevWakeup pin failed (0x%x)",result);
            return CYBT_ERR_GPIO_DEV_WAKE_INIT_FAILED;
        }

        cyhal_gpio_write(p_bt_platform_cfg->controller_config.sleep_mode.device_wakeup_pin,
                         false
                        );
        cy_rtos_delay_milliseconds(100);
    }

    result = cyhal_gpio_init(p_bt_platform_cfg->controller_config.bt_power_pin,
                            BT_POWER_PIN_DIRECTION,
                            BT_POWER_PIN_DRIVE_MODE,
                            BT_POWER_PIN_INIT_VALUE
                            );
     //If the bt_power_pin already initialized by the device configurator or application, we get CYHAL_HWMGR_RSLT_ERR_INUSE.
    if(CYHAL_HWMGR_RSLT_ERR_INUSE == result)
    {
        HCIDRV_TRACE_ERROR("hci_open(): BtPower pin already initialized by the app/configurator -  status: (0x%x)", result);
    }
    //For any other error, we return init failure.
    else if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("hci_open(): Init BtPower pin failed (0x%x)",result);
        return CYBT_ERR_GPIO_POWER_INIT_FAILED;
    }

#ifdef COMPONENT_55500
    cybt_enter_autobaud_mode();
#else
        cyhal_gpio_write(p_bt_platform_cfg->controller_config.bt_power_pin,
                         true
                        );
        cy_rtos_delay_milliseconds(500);
#endif //COMPONENT_55500

    bt_uart_cfg.data_bits = p_bt_platform_cfg->hci_config.hci.hci_uart.data_bits;
    bt_uart_cfg.stop_bits = p_bt_platform_cfg->hci_config.hci.hci_uart.stop_bits;
    bt_uart_cfg.parity = p_bt_platform_cfg->hci_config.hci.hci_uart.parity;
    bt_uart_cfg.rx_buffer = NULL;
    bt_uart_cfg.rx_buffer_size = 0;

#if (CYHAL_API_VERSION >= 2)
    result = cyhal_uart_init(&hci_uart_cb.hal_obj,
                             p_bt_platform_cfg->hci_config.hci.hci_uart.uart_tx_pin,
                             p_bt_platform_cfg->hci_config.hci.hci_uart.uart_rx_pin,
                             p_bt_platform_cfg->hci_config.hci.hci_uart.uart_cts_pin,
                             p_bt_platform_cfg->hci_config.hci.hci_uart.uart_rts_pin,
                             NULL,
                             &bt_uart_cfg
                            );
#else
    result = cyhal_uart_init(&hci_uart_cb.hal_obj,
                             p_bt_platform_cfg->hci_config.hci.hci_uart.uart_tx_pin,
                             p_bt_platform_cfg->hci_config.hci.hci_uart.uart_rx_pin,
                             NULL,
                             &bt_uart_cfg
                            );
#endif
    if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("hci_open(): init error (0x%x)", result);
        return  CYBT_ERR_HCI_INIT_FAILED;
    }

#ifdef COMPONENT_55500
    platform_baud_download=p_bt_platform_cfg->hci_config.hci.hci_uart.baud_rate_for_fw_download;
    result = cyhal_uart_set_baud(&hci_uart_cb.hal_obj,
                                 platform_baud_download,
                                 &actual_baud_rate
                                );
#else
    result = cyhal_uart_set_baud(&hci_uart_cb.hal_obj,
                                     HCI_UART_DEFAULT_BAUDRATE,
                                     &actual_baud_rate
                                    );
#endif
    if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("hci_open(): Set baud rate failed (0x%x)",
                           result
                          );
        return  CYBT_ERR_HCI_SET_BAUDRATE_FAILED;
    }
    HCIDRV_TRACE_DEBUG("hci_open(): act baud rate  = %d", actual_baud_rate);

    if(true == p_bt_platform_cfg->hci_config.hci.hci_uart.flow_control)
    {
    #if (CYHAL_API_VERSION >= 2)
        result = cyhal_uart_enable_flow_control(&hci_uart_cb.hal_obj, true, true);
    #else
        result = cyhal_uart_set_flow_control(&hci_uart_cb.hal_obj,
                                            p_bt_platform_cfg->hci_config.hci.hci_uart.uart_cts_pin,
                                            p_bt_platform_cfg->hci_config.hci.hci_uart.uart_rts_pin
                                           );
    #endif
        if(CY_RSLT_SUCCESS != result)
        {
            HCIDRV_TRACE_ERROR("hci_open(): Set flow control failed (0x%x)",
                               result
                              );
            return  CYBT_ERR_HCI_SET_FLOW_CTRL_FAILED;
        }
    }

    cyhal_uart_register_callback(&hci_uart_cb.hal_obj,
                                 cybt_uart_irq_handler,
                                 NULL
                                );

    cyhal_uart_enable_event(&hci_uart_cb.hal_obj,
                            enable_irq_event,
                            CYHAL_ISR_PRIORITY_DEFAULT,
                            true
                           );

    HCIDRV_TRACE_DEBUG("hci_open(): Wait CTS low");
    while(true == cyhal_gpio_read(p_bt_platform_cfg->hci_config.hci.hci_uart.uart_cts_pin))
    {
        cy_rtos_delay_milliseconds(10);
    }

    hci_uart_cb.inited = true;

    HCIDRV_TRACE_DEBUG("hci_open(): Done");

    return  CYBT_SUCCESS;
}

#ifdef COMPONENT_55500
static void cybt_enter_autobaud_mode(void)
{
    const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();

    /* Initialize UART RTS Pin to be controlled manually */
    cyhal_gpio_init(p_bt_platform_cfg->hci_config.hci.hci_uart.uart_rts_pin,
                    CYHAL_GPIO_DIR_OUTPUT,
                    CYHAL_GPIO_DRIVE_STRONG,
                    true
                   );
    /* Pull RTS line low */
    cyhal_gpio_write(p_bt_platform_cfg->hci_config.hci.hci_uart.uart_rts_pin, false);

    /* Toggle BT REG ON pin */
    cyhal_gpio_write(p_bt_platform_cfg->controller_config.bt_power_pin, false);
    cy_rtos_delay_milliseconds(100);

    cyhal_gpio_write(p_bt_platform_cfg->controller_config.bt_power_pin, true);
    cy_rtos_delay_milliseconds(100);

    /* Release RTS pin to be used by UART block */
    cyhal_gpio_free(p_bt_platform_cfg->hci_config.hci.hci_uart.uart_rts_pin);
}
#endif //COMPONENT_55500

cybt_result_t cybt_platform_hci_set_baudrate(uint32_t baudrate)
{
    uint32_t actual_baud;
    cy_rslt_t result = cyhal_uart_set_baud(&hci_uart_cb.hal_obj, baudrate, &actual_baud);

    if(false == hci_uart_cb.inited)
    {
        HCIDRV_TRACE_ERROR("set_baudrate(): UART is NOT initialized");
        return  CYBT_ERR_HCI_NOT_INITIALIZE;
    }

    if(CY_RSLT_SUCCESS == result)
    {
        HCIDRV_TRACE_DEBUG("set_baudrate(): SUCCESS, req = %d, actual = %d",
                           baudrate,
                           actual_baud
                          );
        return  CYBT_SUCCESS;
    }
    else
    {
        HCIDRV_TRACE_ERROR("set_baudrate(): FAILED(0x%x)", result);
        return  CYBT_ERR_HCI_SET_BAUDRATE_FAILED;
    }
}

cybt_result_t cybt_platform_hci_write(hci_packet_type_t type,
                                                  uint8_t          *p_data,
                                                  uint32_t         length
                                                 )
{
    cy_rslt_t result;
    cybt_result_t return_status =  CYBT_SUCCESS;

    if(false == hci_uart_cb.inited)
    {
        HCIDRV_TRACE_ERROR("hci_write(): UART is NOT initialized");
        return  CYBT_ERR_HCI_NOT_INITIALIZE;
    }

    result = cy_rtos_get_mutex(&hci_uart_cb.tx_atomic, CY_RTOS_NEVER_TIMEOUT);
    if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("hci_write(): Get mutex error (0x%x)\n", result);
        return  CYBT_ERR_HCI_GET_TX_MUTEX_FAILED;
    }

    cybt_platform_sleep_lock();
    cybt_platform_assert_bt_wake();

    result = cyhal_uart_write_async(&hci_uart_cb.hal_obj,
                                    (void *) p_data,
                                    (size_t) length
                                   );
    if(CY_RSLT_SUCCESS == result)
    {
        cy_rtos_get_semaphore(&hci_uart_cb.tx_complete, CY_RTOS_NEVER_TIMEOUT, false);
    }
    else
    {
        HCIDRV_TRACE_ERROR("hci_write(): failure (0x%x)\n", result);
        return_status =  CYBT_ERR_HCI_WRITE_FAILED;
    }

    cybt_platform_deassert_bt_wake();
    cybt_platform_sleep_unlock();

    cy_rtos_set_mutex(&hci_uart_cb.tx_atomic);

    return return_status;
}

cybt_result_t cybt_platform_hci_read(hci_packet_type_t type,
                                                uint8_t           *p_data,
                                                uint32_t          *p_length,
                                                uint32_t          timeout_ms
                                               )
{
    uint32_t  req_len = *p_length;
    cy_rslt_t result;
    cybt_result_t return_status;

    if(false == hci_uart_cb.inited)
    {
        HCIDRV_TRACE_ERROR("hci_read(): UART is NOT initialized");
        return  CYBT_ERR_HCI_NOT_INITIALIZE;
    }

    if (0 == req_len)
    {
        HCIDRV_TRACE_ERROR("hci_read(): Required length = 0\n");
        return  CYBT_ERR_BADARG;
    }

    result = cy_rtos_get_mutex(&hci_uart_cb.rx_atomic, timeout_ms);
    if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("hci_read(): Get mutex error (0x%x)\n", result);
        return  CYBT_ERR_HCI_GET_RX_MUTEX_FAILED;
    }

    cybt_platform_sleep_lock();

    if(0 < timeout_ms)
    {
        result = cyhal_uart_read_async(&hci_uart_cb.hal_obj,
                                       (void *) p_data,
                                       (size_t) req_len
                                      );
        if(CY_RSLT_SUCCESS == result)
        {
            result = cy_rtos_get_semaphore(&hci_uart_cb.rx_complete,
                                           timeout_ms,
                                           false
                                          );
        }
        else
        {
            HCIDRV_TRACE_ERROR("hci_read(): failure code = 0x%x\n", result);

            cybt_platform_sleep_unlock();
            cy_rtos_set_mutex(&hci_uart_cb.rx_atomic);
            return  CYBT_ERR_HCI_READ_FAILED;
        }

        if(CY_RSLT_SUCCESS == result)
        {
            return_status = CYBT_SUCCESS;
        }
        else
        {
            HCIDRV_TRACE_ERROR("hci_read(): failed (0x%x), read size = %d",
                               result,
                               hci_uart_cb.hal_obj.context.rxBufIdx
                              );

            if(CY_RTOS_TIMEOUT == result)
            {
                return_status =  CYBT_ERR_TIMEOUT;
            }
            else
            {
                return_status =  CYBT_ERR_GENERIC;
            }
            *p_length = hci_uart_cb.hal_obj.context.rxBufIdx;

            cyhal_uart_read_abort(&hci_uart_cb.hal_obj);
        }
    }
    else
    {
        result = cyhal_uart_read(&hci_uart_cb.hal_obj,
                                 (void *) p_data,
                                 (size_t *)p_length
                                );
        if(CY_RSLT_SUCCESS == result)
        {
            return_status = CYBT_SUCCESS;
        }
        else
        {
            return_status =  CYBT_ERR_HCI_READ_FAILED;
        }
    }

    cybt_platform_sleep_unlock();
    cy_rtos_set_mutex(&hci_uart_cb.rx_atomic);

    return return_status;

}

cybt_result_t cybt_platform_hci_close(void)
{
    cyhal_uart_event_t enable_irq_event = (cyhal_uart_event_t)(CYHAL_UART_IRQ_RX_DONE
                                           | CYHAL_UART_IRQ_TX_DONE
                                           | CYHAL_UART_IRQ_RX_NOT_EMPTY
                                          );
    const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();
    if(false == hci_uart_cb.inited)
    {
        HCIDRV_TRACE_ERROR("hci_close(): Not inited\n");
        return  CYBT_ERR_HCI_NOT_INITIALIZE;
    }

    cyhal_uart_enable_event(&hci_uart_cb.hal_obj,
                            enable_irq_event,
                            CYHAL_ISR_PRIORITY_DEFAULT,
                            false
                           );
    cyhal_uart_register_callback(&hci_uart_cb.hal_obj,
                                 NULL,
                                 NULL
                                );
    cyhal_uart_free(&hci_uart_cb.hal_obj);

    if((CYBT_SLEEP_MODE_ENABLED == p_bt_platform_cfg->controller_config.sleep_mode.sleep_mode_enabled)
       && (NC != p_bt_platform_cfg->controller_config.sleep_mode.device_wakeup_pin)
       && (NC != p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin)
      )
    {
        cyhal_gpio_enable_event(p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin,
                                CYHAL_GPIO_IRQ_NONE,
                                CYHAL_ISR_PRIORITY_DEFAULT,
                                true
                               );
 #if (CYHAL_API_VERSION >= 2)
        cyhal_gpio_register_callback(p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin, NULL);
#else
        cyhal_gpio_register_callback(p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin,
                                     NULL,
                                     NULL
                                    );
#endif
    }

    if(NC != p_bt_platform_cfg->controller_config.sleep_mode.device_wakeup_pin)
    {
        cyhal_gpio_free(p_bt_platform_cfg->controller_config.sleep_mode.device_wakeup_pin);
    }

    if(NC != p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin)
    {
        cyhal_gpio_free(p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin);
    }

    cy_rtos_deinit_mutex(&hci_uart_cb.tx_atomic);
    cy_rtos_deinit_mutex(&hci_uart_cb.rx_atomic);
    cy_rtos_deinit_semaphore(&hci_uart_cb.tx_complete);
    cy_rtos_deinit_semaphore(&hci_uart_cb.rx_complete);

    cyhal_gpio_write(p_bt_platform_cfg->controller_config.bt_power_pin,
                     false
                    );
    cyhal_gpio_free(p_bt_platform_cfg->controller_config.bt_power_pin);

    memset(&hci_uart_cb, 0, sizeof(hci_uart_cb_t));

    return  CYBT_SUCCESS;
}

void cybt_platform_hci_irq_rx_data_ind(bool enable)
{
    cyhal_uart_enable_event(&hci_uart_cb.hal_obj,
                            CYHAL_UART_IRQ_RX_NOT_EMPTY,
                            CYHAL_ISR_PRIORITY_DEFAULT,
                            enable
                           );
}

#if( configUSE_TICKLESS_IDLE != 0 )
void cybt_idle_timer_cback(cy_timer_callback_arg_t arg)
{
    cybt_platform_disable_irq();

    if (platform_sleep_lock == true)
    {
        cyhal_syspm_unlock_deepsleep();
        platform_sleep_lock = false;
    }

    cybt_platform_enable_irq();
}

cybt_result_t cybt_send_action_to_sleep_task(sleep_action_t action)
{
    bool is_from_isr = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

    cy_rslt_t result = cy_rtos_put_queue(&sleep_timer_task_queue,
                                         (void *)&action,
                                         0,
                                         is_from_isr
                                        );
    UNUSED_VARIABLE(is_from_isr);

    if(CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
        return CYBT_ERR_SEND_QUEUE_FAILED;
    }

    return CYBT_SUCCESS;
}

void cybt_sleep_timer_task(cy_thread_arg_t arg)
{
    cy_rslt_t result;
    sleep_action_t action = 0;

    while(1)
    {
        result = cy_rtos_get_queue(&sleep_timer_task_queue,
                                   (void *)&action,
                                   CY_RTOS_NEVER_TIMEOUT,
                                   false
                                  );

        if(CY_RSLT_SUCCESS != result)
        {
            MAIN_TRACE_WARNING("sleep_task(): queue error (0x%x)",
                                result
                              );
            continue;
        }

        if (SLEEP_ACT_EXIT_SLEEP_TASK == action)
        {
            cy_rtos_deinit_queue(&sleep_timer_task_queue);
            break;
        }

        switch(action)
        {
            case SLEEP_ACT_START_IDLE_TIMER:
                cy_rtos_start_timer(&platform_sleep_idle_timer,
                                    PLATFORM_SLEEP_IDLE_TIMEOUT_MS
                                   );
                break;
            case SLEEP_ACT_STOP_IDLE_TIMER:
                {
                    bool is_timer_running = false;
                    cy_rslt_t result;

                    result = cy_rtos_is_running_timer(&platform_sleep_idle_timer,
                                                      &is_timer_running
                                                     );
                    if(CY_RSLT_SUCCESS == result && true == is_timer_running)
                    {
                        cy_rtos_stop_timer(&platform_sleep_idle_timer);
                    }
                }
                break;
            default:
                MAIN_TRACE_ERROR("sleep_task(): Unknown action = 0x%x", action);
                break;
        }
    }
    cy_rtos_exit_thread();
}

void cybt_platform_terminate_sleep_thread(void)
{
    cy_rslt_t cy_result;

    cy_result = cy_rtos_join_thread(&sleep_timer_task);
    if(CY_RSLT_SUCCESS != cy_result)
    {
        MAIN_TRACE_ERROR("terminate Sleep thread failed 0x%x\n", cy_result);
    }
}

#endif
