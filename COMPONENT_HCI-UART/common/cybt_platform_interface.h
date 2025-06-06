/*******************************************************************************
* \file cybt_platform_interface.h
*
* \brief
* Defines the BT platform porting Interface. Include all interface header
* and provides prototypes for functions that are used in WICED Bluetooth
* library. Functions which are required to be ported are defined
* in cybt_platform_sample.c file.
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

#ifndef CYBT_PLATFORM_INTERFACE_H
#define CYBT_PLATFORM_INTERFACE_H

#include "cybt_result.h"

/**
 *  RTOS interface, including task, mutex, semaphore, queue and timer.
 */
#include "cyabs_rtos.h"

#if defined (CY_USING_HAL)
/**
 *  GPIO interface which is related to pin control of BT chip.
 */
#include "cyhal_gpio.h"
#endif

/**
 *  BT HCI transport interface, which is used to communicate with
 *  BT controller.
 */
#include "cybt_platform_hci.h"


/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define  CYBT_TRACE_BUFFER_SIZE    (128)

/** Define start of the function placed to the SRAM area by the linker */
#if (defined(__ARMCC_VERSION) || defined(__llvm__))
   /** To create cross compiler compatible code, use the CY_NOINIT, CY_SECTION, CY_UNUSED, CY_ALIGN
     * attributes at the first place of declaration/definition.
     * For example: CY_NOINIT uint32_t noinitVar;
     */
   #define BTSTACK_PORTING_SECTION_BEGIN __attribute__((section(".text.cy_btstack_porting")))
   #define BTSTACK_PORTING_SECTION_END

#elif defined(__ICCARM__)
   #define BTSTACK_PORTING_SECTION_BEGIN _Pragma("default_function_attributes = @\".text.cy_btstack_porting\"")
   #define BTSTACK_PORTING_SECTION_END _Pragma("default_function_attributes = ")
#elif defined(__GNUC__)
    #if defined(__clang__)
        #define BTSTACK_PORTING_SECTION_BEGIN __attribute__((section("__DATA, .text.cy_btstack_porting")))
        #define BTSTACK_PORTING_SECTION_END
    #else
        #define BTSTACK_PORTING_SECTION_BEGIN __attribute__((section(".text.cy_btstack_porting")))
        #define BTSTACK_PORTING_SECTION_END
    #endif
#else // if defined(__ARMCC_VERSION)
    #define BTSTACK_PORTING_SECTION_BEGIN
    #define BTSTACK_PORTING_SECTION_END
#endif // (__ARMCC_VERSION)

#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************
 *                           Function Declarations
 *****************************************************************************/

/**
 * The first platform-port function to be invoked. Initialization for
 * everything (e.g. os components, peripheral driver, timer, etc.) can
 * be put inside this function.
 *
 * @returns  void
 */
void cybt_platform_init(void);


/**
 * The platform-port function which is used to de-initialization all
 * the components (e.g. os components, peripheral driver, timer, etc.) which
 * had been intialized for Bluetooth.
 * It will be invoked when BT stack shutdown API is called.
 *
 * @returns  void
 */
void cybt_platform_deinit(void);


/**
 * Get memory via OS malloc function.
 *
 * @param[in] req_size: the requested size of memory
 *
 * @returns the pointer of memory block
 */
void *cybt_platform_malloc(uint32_t req_size);


/**
 * Return memory to OS via OS free function.
 *
 * @param[in] p_mem_block: the pointer of memory block which was allocated
 *                          by cybt_platform_malloc() function.
 *
 * @return  void
 */
void cybt_platform_free(void *p_mem_block);


/**
 * Disable system interrupt.
 *
 * @return  void
 */
void cybt_platform_disable_irq(void);


/**
 * Enable system interrupt.
 *
 * @return  void
 */
void cybt_platform_enable_irq(void);


/**
 * Log printing function. It will be invoked whenever stack has log output.
 * In this function these logs can be forwarded to UART, log task, file system,
 * or somewhere else, depends on the implementation.
 *
 * @param[in] fmt_str :  output trace string
 *
 * @return  void
 */
void cybt_platform_log_print(const char *fmt_str, ...);


/**
 * Prevent the system from entering sleep mode.
 *
 * @return  void
 */
void cybt_platform_sleep_lock(void);


/**
 * Allow the system to enter sleep mode.
 *
 * @return void
 */
void cybt_platform_sleep_unlock(void);


/**
 * This function is used by BT stack to get current tick count.
 *  The unit is micro-second.
 *
 * @return the current tick count in micro-second
 */
uint64_t cybt_platform_get_tick_count_us(void);


/**
 * This function is used by BT stack to set next timeout in absolute tick
 * count in micro-second.
 *
 * @param[in] abs_tick_us_to_expire: absolute tick count in micro-second to be expired
 *
 * @return
 */
void cybt_platform_set_next_timeout(uint64_t abs_tick_us_to_expire);


/**
* Call back to the application in the BT stack context
*
* @param[in] void
*
* @returns  void
*/
void cybt_call_app_in_stack_context(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif

