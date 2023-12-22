Contains FW patch for 0dBm TX Power.

This component should be used with non-PAWR and non-ISOC applications. To include this component in application, add the lines provided below to application makefile:
- DISABLE_COMPONENTS+= BTFW-TX10
- COMPONENTS+= BTFW-TX0


© Infineon Technologies, 2023.

