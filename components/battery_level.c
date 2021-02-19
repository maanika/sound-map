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
*   Private Function Declarations
*******************************************************************************/
static float readBatteryVoltage();

/*******************************************************************************
*   Variable definitions
*******************************************************************************/
/* for debugging - uncomment when in need */
static char tempStr[100];

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

/*******************************************************************************
* Function Name: readBatteryVoltage
********************************************************************************
* Summary:
*   Reads battery voltage.
*******************************************************************************/
static float readBatteryVoltage()
{
    int16 adcResult = 0;
    float volts = 0.00;
    
    ADC_Battery_StartConvert();
    
    /* Wait for get ADC converted value */
    if(ADC_Battery_IsEndConversion(ADC_Battery_WAIT_FOR_RESULT))
    {
        adcResult = ADC_Battery_GetResult16();
        
        /* convert value to Volts */
        volts = ADC_Battery_CountsTo_Volts(adcResult) / 0.5735;
        
        /* for testing */
        sprintf(tempStr, "Battery volts: %.2f\n", volts);
        UART_PutString(tempStr);
    }
    
    return volts;
}
