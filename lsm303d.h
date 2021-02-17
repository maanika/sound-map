/*******************************************************************************
* Written by : Maanika Kenneth Koththigoda (kennethmaanika@gmail.com)
* Last Modified : 25/10/2020
*
* @file    lsm303d.h
* @version 1.0.0
*
* @brief Library for PSoC devices that interfaces with LSM303D compass
*        and accelerometer ICs on Pololu boards. It allows to read the raw 
*        accelerometer and magnetometer data from LSM303D
*
* @par Target device
*    CY8C5888LTI - LP097
*
* @par Code Tested With
*    - Silicon: PSoC 5LP
*    - IDE: PSoC Creator 4.3
*    - Compiler: GCC 5.4
*
* @par Components
*    - SDA_1       [I2C_1]
*    - SCL_1       [I2C_1]
*    - RST_1       [I2C_1]
*    - Clock_I2C_1 [I2C_1]
*
* @par Source
*    LSM303D library code for PSoC 5LP is adapted from https://github.com/pololu/lsm303-arduino
*
*    I2C Read and Write functions using low level API 
*    is taken from https://community.cypress.com/docs/DOC-15336

* @note
*   Calibration values are by default min{-7733, -4797, -2535} 
*   and max{ -1979, +875, +642}
*   Use the Calibrate example program from Arduino to determine 
*   appropriate values for your particular unit
*
*******************************************************************************/
#include "project.h"

/*******************************************************************************
*   Function Declarations
*******************************************************************************/

void compassStart();


// @brief Initializes all accelerometer and magnetometer, Sets values to defaults
// @return '1' if sucess or '0' if fail
void compassInitialize();

// @brief Calculates the magnetic heading with respect to the y axis
// @return heading value
double heading(void);

/* [] END OF FILE */