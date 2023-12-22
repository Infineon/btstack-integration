Contains FW patch with WBMS support for 10dBm TX Power.

This component should be used with WBMS application. To include this component in application, add the lines provided below to application makefile:
- DISABLE_COMPONENTS+= BTFW-TX10
- COMPONENTS+= BTFW-WBMS-TX10


© Infineon Technologies, 2023.
