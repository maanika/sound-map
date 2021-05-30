/*******************************************************************************
* Written by : Maanika Kenneth Koththigoda
* Last Modified : 15/03/2021
*
* File:    motor.h
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
********************************************************************************
*   Function Declarations
*******************************************************************************/
// Brief: Start PWM components.
// Param: none.
// Return: none.
void startMotors();

// Brief: Set the Duty cycle according to the passed parameters.
// Param: Distances from Ultrasonic readings
// Return: none
void setMotors(int num1, int num2, int num3);

/* [] END OF FILE */
