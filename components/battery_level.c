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
#include <battery_level.h>
#include <mode.h>

/*******************************************************************************
*   Private Function Declarations
*******************************************************************************/
static float readBatteryVoltage();

/*******************************************************************************
*   Variable definitions
*******************************************************************************/
/* for debugging - uncomment when in need */
static char temp[100];

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
        volts = ADC_Battery_CountsTo_Volts(adcResult) / VOLTAGE_DIVIDER_RATIO;

        #if DEBUG_PRINT_MODE == 1
            //sprintf(temp, "Battery volts: %.2f\n", volts);
            //UART_PutString(temp);
        #endif
    }
    return volts;
}

/*******************************************************************************
* Function Name: batteryLevel
********************************************************************************
* Summary
*   Matched battery voltage to appropriate percentage level.
*******************************************************************************/
int readBatteryLevel()
{
    float volts = 0.0;
    volts = readBatteryVoltage();

    if ( volts > 8.40 || volts <= 6.00 ) return 0; /* 0 indicates an error */
    else if (volts < 8.40 && volts >= 8.16) return 90;
    else if (volts < 8.16 && volts >= 7.92) return 80;
    else if (volts < 7.92 && volts >= 7.68) return 70;
    else if (volts < 7.68 && volts >= 7.44) return 60;
    else if (volts < 7.44 && volts >= 7.20) return 50;
    else if (volts < 7.20 && volts >= 6.96) return 50;
    else if (volts < 6.96 && volts >= 6.72) return 40;
    else if (volts < 6.72 && volts >= 6.48) return 30;
    else if (volts < 6.48 && volts >= 6.24) return 20;
    else return 10;
}
/* [] END OF FILE */
