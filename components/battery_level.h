/*******************************************************************************
* Written by Maanika Kenneth Koththigoda, for PSoC5LP
* Last Modified on 19/02/2021
*
* File:     battery_level.h
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
*******************************************************************************/
#ifndef BATTERY_LEVEL_H
	#define BATTERY_LEVEL_H
    
/*******************************************************************************
*   Included Headers
*******************************************************************************/
    #include <project.h>
    
/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/
    #define VOLTAGE_DIVIDER_RATIO 0.5735
    
/*******************************************************************************
*   Function Declarations
*******************************************************************************/
    // Brief: Start components for battery level monitoring.
    // Param: none.
    // Return: none
    void batteryLevelMonitorStart();
    
    // Brief: Converts battery voltage to a percentage level.
    // Param: none.
    // Return: battery percentage.
    int readBatteryLevel();
    
#endif
    
/* [] END OF FILE */