/*******************************************************************************
* Written by : Maanika Kenneth Koththigoda
* Last Modified : 4/11/2020
*
* File:    distance.c
* Version: 1.0.0
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
*    Reading from analog pins is taken from this guide
*    https://www.esipfed.org/student-fellow-blog/diy-sensing-anatomy-of-an-analog-sensor
*    Changing method is adapted from the datasheet of the Ultrasonic sensor
*
*******************************************************************************/
#include <project.h>
#include "distance.h"

/*******************************************************************************
*   Constant definitions
*******************************************************************************/
#define MAXBOTIX_V_RANGE 5

/* Sensor Resolution = Vcc / 1024, where Vcc is 4.93 here. */
#define MAXBOTIX_RESOLUTION 4.84375e-3

/*******************************************************************************
* Function Name: distanceReading
********************************************************************************
* Summary:
*   Reads analog volage from Ultrasonic Sensors and uses a VDAC to get distance.
*   Reads three sensors in an output comman loops
*******************************************************************************/
void startUltrasonicSensors( void )
{
    ADC_SAR_Seq_1_Start();
}
/*******************************************************************************
* Function Name: distanceReading
********************************************************************************
* Summary:
*   Reads analog volage from Ultrasonic Sensors and uses a VDAC to get distance
*   in cm.
*   Reads three sensors in an output comman loops
*******************************************************************************/
void distanceReading(ultrasonicSensor *readings)
{
    // Initialize variables
    int16 adc_result1 = 0;
    int16 adc_result2 = 0;
    int16 adc_result3 = 0;
    
    float volt_reading1 = 0;
    float volt_reading2 = 0;
    float volt_reading3 = 0;
    
    /* Start conversion */
    ADC_SAR_Seq_1_StartConvert();
    
    Ultrasonic_Rx_Write(1);
    CyDelayUs(20);
    Ultrasonic_Rx_Write(0);
    
    /* Sensor 1 */
    /* If ADC has finished converting, get the result */
    adc_result1 = ADC_SAR_Seq_1_GetResult16(0);
    
    /* Convert the raw ADC reading to volts */
    volt_reading1 = ADC_SAR_Seq_1_CountsTo_Volts(adc_result1);
    
    /* Convert voltage to distance */
    readings->distance1 = volt_reading1 * (MAXBOTIX_V_RANGE / MAXBOTIX_RESOLUTION) * 0.1;
    
    /* Sensor 2 */
    adc_result2 =  ADC_SAR_Seq_1_GetResult16(1);
    volt_reading2 =  ADC_SAR_Seq_1_CountsTo_Volts(adc_result2);
    readings->distance2 = volt_reading2 * (MAXBOTIX_V_RANGE / MAXBOTIX_RESOLUTION) * 0.1;
    
    /* Sensor 3 */
    adc_result3 =  ADC_SAR_Seq_1_GetResult16(2);
    volt_reading3 =  ADC_SAR_Seq_1_CountsTo_Volts(adc_result3);
    readings->distance3 = volt_reading3 * (MAXBOTIX_V_RANGE / MAXBOTIX_RESOLUTION) * 0.1;
    
    /* End conversion */
    ADC_SAR_Seq_1_StopConvert();
}

/* [] END OF FILE */