Contains FW patch with ISOC support for 10dBm TX Power.

This component should be used with ISOC applications. To include this component in application, add the lines provided below to application makefile:
- DISABLE_COMPONENTS+= BTFW-TX10
- COMPONENTS+= BTFW-ISOC-TX10


© Infineon Technologies, 2023.
