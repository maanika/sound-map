/*******************************************************************************
* Written by : Maanika Kenneth Koththigoda (kennethmaanika@gmail.com)
* Last Modified : 4/11/2020
*
* File: distance.c
* version: 1.0.0
*
* Brief: for PSoC devices that interfaces HRLV-MaxSonar-EZ0. Takes readings of two
*        takes two sensor readings in an output command loop.
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
*   ADC_SAR_1
*   ADC_SAR_2
*   UART_1
*
* Source:
*     Reading from analog pins is taken from this guide
*    https://www.esipfed.org/student-fellow-blog/diy-sensing-anatomy-of-an-analog-sensor
*
*******************************************************************************/

typedef struct ultrasonicSensor{
    double distance1;
    double distance2;
    double distance3;
} ultrasonicSensor;


/*******************************************************************************
*   Function Declarations
*******************************************************************************/

void startUltrasonicSensors( void );

// brief Takes Distance Readings from two HRLV-MaxSonar-EZ0 sensors
// return distance readings
void distanceReading( ultrasonicSensor *readings);


/* [] END OF FILE */
