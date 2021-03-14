/*******************************************************************************
* Written by : Maanika Kenneth Koththigoda
* Last Modified : 15/03/2021
*
* File:    motor.c
* Version: 1.0.0
*
* Brief: Start motors and turn them on according to the values passed.
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
*   PWM_1
*   PWM_2
*   Clock_PWM
*
*******************************************************************************/
#include <project.h>
#include "stdio.h"
#include "motor.h"

/*******************************************************************************
*   Constant definitions
*******************************************************************************/
#define PWM_PERIOD      150
#define MAX_DISTANCE    150

/*******************************************************************************
*   Variables
*******************************************************************************/
static char tempString[20];

/*******************************************************************************
* Function Name: startMotors
********************************************************************************
* Summary:
*   Start PWM components.
*******************************************************************************/
void startMotors()
{
    PWM_1_Start();
    PWM_2_Start();
}

/*******************************************************************************
* Function Name: setMotors
********************************************************************************
* Summary:
*   Set the pulse width according to the passed parameters.
*******************************************************************************/
void setMotors(int distance1, int distance2, int distance3)
{
    if ( distance1 <= MAX_DISTANCE )
    {
        PWM_1_WriteCompare1( PWM_PERIOD - distance1 );
        sprintf( tempString, "num1: %d       Pulse width1: %d\n", distance1, PWM_PERIOD - distance1 );
        UART_1_PutString( tempString );
    }
    else
    {
        PWM_2_WriteCompare( 0 );
    }
    if ( distance2 <= MAX_DISTANCE )
    {
        PWM_1_WriteCompare2( PWM_PERIOD - distance2 );
        sprintf( tempString, "num2: %d       Pulse width2: %d\n", distance2, PWM_PERIOD - distance2 );
        UART_1_PutString( tempString );
    }
    else
    {
        PWM_1_WriteCompare2(0);
    }
    if ( distance3 <= MAX_DISTANCE )
    {
        PWM_2_WriteCompare( PWM_PERIOD - distance3 );
        sprintf( tempString, "num3: %d       Pulse width3: %d\n", distance3, PWM_PERIOD - distance3 );
        UART_1_PutString( tempString );
    }
    else
    {
        PWM_1_WriteCompare1(0);
    }
    return;
}


/* [] END OF FILE */
