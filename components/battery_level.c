/*******************************************************************************
* Written by Maanika Kenneth Koththigoda, for PSoC5LP
* Last Modified on 19/02/2021
*
* File:     battery_level.c
* Version:  1.0.0
*
* Brief: Monitors the battery level.
*
* Target device:
*    CY8C5888LTI - LP097
*
* Code Tested With:
*    - Silicon: PSoC 5LP
*    - IDE: PSoC Creator 4.3
*    - Compiler: GCC 5.4
*
* Components:
*    - ADC_SAR       [ADC_Battery]
*    - cy_pins       [Battery_pin]
*
*******************************************************************************
*   Included Headers
*******************************************************************************/
#include <project.h>
#include <stdio.h>

/*******************************************************************************
* Function Name: batteryLevelMonitorStart
********************************************************************************
* Summary:
*  Start required components for battery level monitoring.
*******************************************************************************/
void batteryLevelMonitorStart()
{
    ADC_Battery_Start();
}
