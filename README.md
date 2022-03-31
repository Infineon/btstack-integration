# btstack-integration Overview


The btstack-integration hosts platform adaptation layer (porting layer) between AIROC™ BT Stack and Cypress Abstraction Layers (CYHAL and CYOSAL) for different cypress hardware platforms. This layer implements/invokes the interfaces defined by BTSTACK which is a platform agnostic library, to provide OS services and to enable communication with the BT controller.

While these adaptation layer interfaces and functionality are the same, the hardware platform they run on differ in the inter processor communication mechanisms used for communication between host stack and controller.

This asset provides three components, COMPONENT_BLESS-IPC, COMPONENT_BTSS-IPC and COMPONENT_HCI-UART porting layer components for various hardware platforms (such as psoc6-bless, 20829, psoc6+43xx respectively) and IPC methods (IPC_PIPE, IPC_BTSS and UART respectively) supported. Below table points to further documentation for using each of these components.

Please refer below each COMPONENT's README for more details.

| Component's name | Hardware platforms | IPC Method | IPC Method link | Portinglayer documentation link |
| :-----------------: | :----------------: | :--------: | :------------: | :-----------------------------: |
| COMPONENT_BLESS-IPC | psoc6-bless | IPC_PIPE| https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/group__group__ipc__pipe.html | https://git-ore.aus.cypress.com/repo-staging/btstack-integration/-/blob/latest-v4.X/COMPONENT_BLESS-IPC/README.md |
| COMPONENT_BTSS-IPC | psoc6-20829 | IPC_BTSS| https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/group__group__ipc__bt.html | https://git-ore.aus.cypress.com/repo-staging/btstack-integration/-/blob/latest-v4.X/COMPONENT_BTSS-IPC/README.md |
| COMPONENT_HCI-UART | psoc6-43xx | UART| https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/group__group__scb__uart.html | https://git-ore.aus.cypress.com/repo-staging/btstack-integration/-/blob/latest-v4.X/COMPONENT_HCI-UART/README.md |

please refer IPC (Inter Process Communication) section of https://git-ore.aus.cypress.com/repo-staging/mtb-pdl-cat1/-/blob/develop/docs/pdl_api_reference_manual.html for documentation of IPC mechanisms.
    
© Cypress Semiconductor Corporation, 2022.