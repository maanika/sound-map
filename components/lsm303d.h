/*******************************************************************************
* Written by : Maanika Kenneth Koththigoda (kennethmaanika@gmail.com)
* Last Modified : 13/03/2021
*
* @file    LSM303D.c
* @version 1.0.0
*
* @brief Library for PSoC compatible devices that interfaces with LSM303D compass 
*        and accelerometer ICs on Adafruit boards. It allows to read the raw 
*        accelerometer and magnetometer data from LSM303DHLC.
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
*    I2C Read and Write functions using low level API 
*    is taken from https://community.cypress.com/docs/DOC-15336
*
*******************************************************************************/
#include "project.h"

typedef struct compassRaw{
    double m_x;
    double m_y;
    double m_z;
    double a_x;
    double a_y;
    double a_z;
} compassRaw;

/*******************************************************************************
*   Function Declarations
*******************************************************************************/
// @brief Initializes all accelerometer and magnetometer, Sets values to defaults
// @return '1' if sucess or '0' if fail
void compassStart();
    
// @brief Reads all magnetometer and accelerometer data registers
// @param return varaibles
void compassRead(compassRaw *compassData);

/* [] END OF FILE */