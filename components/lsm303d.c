/*******************************************************************************
* Written by : Maanika Kenneth Koththigoda
* Last Modified : 13/03/2021
*
* @file    LSM303D.c
* @version 1.0.0
*
* @brief Library for PSoC compatible devices that interfaces with LSM303DHLC compass 
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
*******************************************************************************
*   Included Headers
*******************************************************************************/
#include "project.h"
#include "lsm303d.h"
#include "math.h"
#include "stdio.h"

/*******************************************************************************
*   Constant definitions
*******************************************************************************/
// Pi Constant
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Sub adress length
#define SUB_ADDRESS_LENGTH (0x01u)

//  Length of data to be read.
#define READ_DATA_LENGTH (1u)

// Length of data to be read, includes register and data
#define WRITE_DATA_LENGTH (2u)

// Axis values
#define XAXIS 0x00
#define YAXIS 0x01
#define ZAXIS 0x02

/*******************************************************************************
* Function Name: I2C_Read
********************************************************************************
* @par Summary
*   The Master device returns the data read from the Slave's sub register 
*   given by the address 'registerAddress'
*******************************************************************************/
static uint8 I2C_Read(uint8 slaveAddress, uint8 registerAddress) {
    
    uint8 i = 0;
    uint8 Status = 0;

    // Load the write buffer with the sub-address location
    uint8 write_data_buffer[SUB_ADDRESS_LENGTH] = { registerAddress };

    // Initialize the read buffer
    uint8 read_data_buffer[READ_DATA_LENGTH];

    // Send start condition and slave address, followed by write bit
    Status = I2C_1_MasterSendStart(slaveAddress, I2C_1_WRITE_XFER_MODE);     

    // Check if the master status is error free
    if(Status == I2C_1_MSTR_NO_ERROR)
    {
        for(i=0; i< SUB_ADDRESS_LENGTH; i++)
        {
            // Send the sub-address
            Status = I2C_1_MasterWriteByte(write_data_buffer[i]);

            // Check if the master status is error free
            if(Status != I2C_1_MSTR_NO_ERROR)
            {
                break;
            }
        }
        // Send start condition and slave address, followed by read bit
        Status = I2C_1_MasterSendRestart(slaveAddress, I2C_1_READ_XFER_MODE);

         // Check if the master status is error free
        if(Status == I2C_1_MSTR_NO_ERROR)
         {
            for(i=0; i < (READ_DATA_LENGTH -1); i++)
            {  
                // Load the read buffer with the data read from slave, followed by ACK
                read_data_buffer[i] = I2C_1_MasterReadByte(I2C_1_ACK_DATA);
            }

            // Load the read buffer with the last data read from slave, followed by NACK     
            read_data_buffer[i] = I2C_1_MasterReadByte(I2C_1_NAK_DATA);
        }      
    }
    // Send stop condition after the transaction is completed
    I2C_1_MasterSendStop();
    
    return * read_data_buffer;
}

/*******************************************************************************
* Function Name: I2C_Write
********************************************************************************
* @par Summary
*   The Master device writes the data given by 'wrData' to the Slave's sub regsister 
*   given by 'registerAddress'
*******************************************************************************/
static void I2C_Write(uint8 slaveAddress, uint8 registerAddress, uint8 wrData) {
    
    uint8 i = 0;
    uint8 Status = 0;
    
    // Load the write buffer with the sub-address location and data to be transmitted
    uint8 write_data_buffer[WRITE_DATA_LENGTH] = { registerAddress , wrData };
    
    // Send start condition and slave address, followed by write bit
    Status = I2C_1_MasterSendStart(slaveAddress, I2C_1_WRITE_XFER_MODE);
    
     // Check if the master status is error free
     if(Status == I2C_1_MSTR_NO_ERROR)
     {
        for(i=0; i<WRITE_DATA_LENGTH; i++)
        {
            // Transmit data to be written to the slave that was loaded into the write buffer initially
            Status = I2C_1_MasterWriteByte(write_data_buffer[i]);
            
            // Check if the master status is error free
            if(Status != I2C_1_MSTR_NO_ERROR)
            {
                break;
            }
        }
    }
    // Send stop condition after the transaction is completed
    I2C_1_MasterSendStop();
}


/*******************************************************************************
* Function Name: Compass_Initialize
********************************************************************************
* @par Summary
*   Enables the LSM303DHLC's accelerometer and magnetometer. 
*   Sets the sensor's full scales (gain) to default power-on values, 
*   which are +/- xxx g for accelerometer and +/- xxxx gauss.
*   Selects xxx Hz ODR (output data rate) for accelerometer and xxx Hz
*   ODR for magnetometer (These are the ODR settings for which the 
*   electrical characteristics are specified in the datasheets).
*   Enables high resolution modes (if available).
*
*******************************************************************************/
void compassStart() {
    
    I2C_1_Start();
    
    I2C_Write(0x19, 0x20, 0x27u);//set CTRL_REG1_A register
    I2C_Write(0x19, 0x23, 0x08u);//set CTRL_REG4_A register
    
    I2C_Write(0x1E, 0x00, 0x0Cu);//set CRA_REG_M register
    I2C_Write(0x1E, 0x01, 0x20u); // SET CRB_REG_M 
    I2C_Write(0x1E, 0x02, 0x00u);//set MR_REG_M register
}

/*******************************************************************************
* Function Name: accelerometer
********************************************************************************
* @par Summary
*   Reads the acceleration data from the selected register given by axis.
*******************************************************************************/
static short accelerometer(int axis){

    unsigned char ACC_Data[2];
    short Raw;
    
    if (axis == 0) {
        ACC_Data[0] = I2C_Read(0x19, 0x28);//read OUT_X_L_A (MSB)
        ACC_Data[1] = I2C_Read(0x19, 0x29);//read OUT_X_H_A (LSB)
    }
    else if (axis == 1) {
        ACC_Data[0] = I2C_Read(0x19, 0x2A);//read OUT_Y_L_A (MSB)
        ACC_Data[1] = I2C_Read(0x19, 0x2B);//read OUT_Y_H_A (LSB)
    }
    else if (axis == 2) {
        ACC_Data[0] = I2C_Read(0x19, 0x2C);//read OUT_Z_L_A (MSB)
        ACC_Data[1] = I2C_Read(0x19, 0x2D);//read OUT_Z_H_A (LSB)
    }
    
    Raw = (short) (( ((unsigned short) ACC_Data[0]) << 8) + ((unsigned short) ACC_Data[1]));
    return Raw;
}

/*******************************************************************************
* Function Name: magnetometer
********************************************************************************
* @par Summary
*   Reads the magnetic data from the selected register given by axis.
*******************************************************************************/
static short magnetometer(int axis){
    
    unsigned char MR_Data[2];
    short Raw;
    
    if (axis == 0) {
        MR_Data[0] = I2C_Read( 0x1E, 0x03 );//read OUT_X_H_M (MSB)
        MR_Data[1] = I2C_Read( 0x1E, 0x04 );//read OUT_X_L_M (LSB)
    }
    else if (axis == 1) {
        MR_Data[0] = I2C_Read( 0x1E, 0x07 );//read OUT_Y_H_M (MSB)
        MR_Data[1] = I2C_Read( 0x1E, 0x08 );//read OUT_Y_L_M (LSB)
    }
    else if (axis == 2) {
        MR_Data[0] = I2C_Read( 0x1E, 0x05 );//read OUT_Z_H_M (MSB)
        MR_Data[1] = I2C_Read( 0x1E, 0x06 );//read OUT_Z_L_M (LSB)
    }
    
    Raw = (short) (( ((unsigned short) MR_Data[0]) << 8) + ((unsigned short) MR_Data[1]));
    return Raw;
}

/*******************************************************************************
* Function Name: compassRead
********************************************************************************
* @par Summary
*   Reads both the magnetic and acceleration data from all axes.
*******************************************************************************/
void compassRead(compassRaw *compassData)
{   
    // magnetometer readings
    compassData->m_x = magnetometer(XAXIS);
    compassData->m_y = magnetometer(YAXIS);
    compassData->m_z = magnetometer(ZAXIS);
   
    // accelerometer readings
    compassData->a_x = accelerometer(XAXIS);
    compassData->a_y = accelerometer(YAXIS);
    compassData->a_z = accelerometer(ZAXIS);   
}

/* [] END OF FILE */
