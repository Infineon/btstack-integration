### AIROC&trade; BT/BLE Host Stack solution
AIROC&trade; BT/BLE stack provides Bluetooth&reg; functionality with high performance and less resource consumption. It targets on IoT products, especially for embedded devices.

### What's Included?
This release of AIROC&trade; BT/BLE stack solution package includes as following:
* Infineon Bluetooth&reg; chip firmware (CYW4343W,CYW43012, 20829)
* Platform and Operating system porting layers for P6+43xx, 20829 and P6+BLESS.

#### v4.0.0
* Initial release of btstack-integration which has HCI-UART, BLESS-IPC & BTSS-IPC components

| Component | Hardware Platform | Description |
| :-------: | :---------------: | :---------: |
| COMPONENT_BLESS-IPC | P6 + BLESS | Represents BLESS protocol talking over IPC |
| COMPONENT_BTSS-IPC | 20829 | Represents BTSS protocol talking over IPC |
| COMPONENT_HCI-UART | P6 + 43xx | Represents HCI as the protocol talking over UART transport |

bluetooth-freertos v3.4.0 has been included as COMPONENT_HCI-UART in btstack-integration. Hence btstack-integration has initial version as 4.0.

### Supported Software and Tools
This version of Infineon BT/BLE stack API was validated for compatibility with the following Software and Tools:

| Software and Tools                        | Version |
| :---                                      | :----:  |
| ModusToolbox™ Software Environment        | 2.4.0   |
| GCC Compiler                              | 10.3.1  |
| IAR Compiler                              | 8.4     |
| ARM Compiler                              | 6.11    |
| mbed OS                                   | 5.15    |
| FreeRTOS                                  | 10.4.3  |

---
© Infineon Technologies, 2019.