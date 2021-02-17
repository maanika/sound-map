/*******************************************************************************
* Written by : Maanika Kenneth Koththigoda (kennethmaanika@gmail.com)
* Last Modified : 25/10/2020
*
* @file    LSM303D.c
* @version 1.0.0
*
* @brief Library for PSoC compatible devices that interfaces with LSM303D compass 
*        and accelerometer ICs on Pololu boards. It allows to read the raw 
*        accelerometer and magnetometer data from LSM303D.
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
*    LSM303D library code for PSoC 5LP is adapted from 
*    https://github.com/pololu/lsm303-arduino
*
*    I2C Read and Write functions using low level API 
*    is taken from https://community.cypress.com/docs/DOC-15336

* @note
*   Calibration values are by default m_min_x = -7733; m_min_y =  -4797;
*   m_min_z = -2535; m_max_x = -1979; m_max_y = +875; m_max_z = +642;
*   Use the Calibrate example program from Arduino to determine 
*   appropriate values for your particular unit.
*
*******************************************************************************/
#include "project.h"
#include "lsm303d.h"

#include "math.h"
#include "stdio.h"

/*******************************************************************************
*   Constant definitions
*******************************************************************************/
// @brief Pi Constant
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// @brief Sub adress length
#define SUB_ADDRESS_LENGTH (0x01u)

//  @brief Length of data to be read.
#define READ_DATA_LENGTH (1u)

// @brief Length of data to be read, includes register and data
#define WRITE_DATA_LENGTH (2u)

// @brief Axis values
#define XAXIS 0x00
#define YAXIS 0x01
#define ZAXIS 0x02

// @brief LSM303D Regsiters



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
* Function Name: compassStart
********************************************************************************
* Summary
*******************************************************************************/
void compassStart() {
    
    I2C_1_Start();
    
}
/*******************************************************************************
* Function Name: compassInitialize
********************************************************************************
* @par Summary
*   Enables the LSM303's accelerometer and magnetometer. 
*   Sets the sensor's full scales (gain) to default power-on values, 
*   which are +/- 2 g for accelerometer and +/- 4 gauss.
*   Selects 50 Hz ODR (output data rate) for accelerometer and 6.25 Hz
*   ODR for magnetometer (These are the ODR settings for which the 
*   electrical characteristics are specified in the datasheets).
*   Enables high resolution modes (if available).
*   Note that this function will also reset other settings controlled by
*   the registers it writes to.
*******************************************************************************/
void compassInitialize() {
    
    //I2C_Write(0x18, 0x20, 0x27);//set CTRL_REG1_A register
    //I2C_Write(0x18, 0x23, 0x40);//set CTRL_REG4_A register
    I2C_Write(0x1E, 0x00, 0x14);//set CRA_REG_M register
    I2C_Write(0x1E, 0x02, 0x00);//set MR_REG_M register
    
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
    
    if (axis==0) {
        ACC_Data[0] = I2C_Read(0x18, 0x28);//read OUT_X_L_A (MSB)
        ACC_Data[1] = I2C_Read(0x18, 0x29);//read OUT_X_H_A (LSB)
    }
    else if (axis==1) {
        ACC_Data[0] = I2C_Read(0x18, 0x2A);//read OUT_Y_L_A (MSB)
        ACC_Data[1] = I2C_Read(0x18, 0x2B);//read OUT_Y_H_A (LSB)
    }
    else if (axis==2) {
        ACC_Data[0] = I2C_Read(0x18, 0x2C);//read OUT_Z_L_A (MSB)
        ACC_Data[1] = I2C_Read(0x18, 0x2D);//read OUT_Z_H_A (LSB)
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
    
    if (axis==0) {
        MR_Data[0] = I2C_Read(0x1E, 0x03 );//read OUT_X_H_M (MSB)
        MR_Data[1] = I2C_Read(0x1E, 0x04);//read OUT_X_L_M (LSB)
    }
    else if (axis==1) {
        MR_Data[0] = I2C_Read(0x1E, 0x07);//read OUT_Y_H_M (MSB)
        MR_Data[1] = I2C_Read(0x1E, 0x08);//read OUT_Y_L_M (LSB)
    }
    else if (axis==2) {
        MR_Data[0] = I2C_Read(0x1E, 0x05);//read OUT_Z_H_M (MSB)
        MR_Data[1] = I2C_Read(0x1E, 0x06);//read OUT_Z_L_M (LSB)
    }
    
    Raw = (short) (( ((unsigned short) MR_Data[0]) << 8) + ((unsigned short) MR_Data[1]));
    return Raw;

}
/*******************************************************************************
* Function Name: readMag
********************************************************************************
* @par Summary
*   Reads the magnetic data from all axes.
*******************************************************************************/
static void readMag(double *return_m_x, double *return_m_y, double *return_m_z)
{
    double m_x, m_y, m_z;
    
    *return_m_x = magnetometer(XAXIS);
    *return_m_y = magnetometer(YAXIS);
    *return_m_z = magnetometer(ZAXIS);
    
}
/*******************************************************************************
* Function Name: heading
********************************************************************************
* @par Summary
*   Returns the angular difference in the horizontal plane between a
*   default vector and north, in degrees.
*   The default vector here is chosen to point along the surface of the
*   PCB, in the direction of the top of the text on the silkscreen.
*   This is the +X axis on the Pololu LSM303D carrier.
*******************************************************************************/
double heading()
{
    double m_x, m_y, m_z;
    readMag(&m_x, &m_y, &m_z);
    
    double heading = (atan2(-m_y, m_x)); //* (180 / M_PI);
    
    if (heading < 0) heading = heading + (2*M_PI);
    return heading;
}

/* [] END OF FILE */