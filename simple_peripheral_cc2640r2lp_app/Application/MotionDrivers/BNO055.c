/*
 * Copyright (c) 2018 MedEngine GmbH
 * All rights reserved.
 *
 * LSMDS1 Driver
 *
 *
 *
 */

#include <MotionDrivers/BNO055.h>
#include "MEi2c.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/PIN.h>
#include "Board.h"


static bool WOM;


 bool isWomEnabled()
 {
     if (WOM == true)
     {
         return true;
     }

     return false;
 }


#define delay_ms(i) Task_sleep( ((i) * 1000) / Clock_tickPeriod )

bool init()
{
    if (!MEinitI2c())
    {
        return false;
    }

    return true;
}

uint8_t _whoAmI()
{
    uint8_t data = 0, buffer = 0;
    startTransaction(BNO055_ADDRESS_ACCELGYRO);
    MEReadReg(BNO055_CHIP_ID, &buffer, 1);
    if (buffer  > 0)
    {
        data++;
    }
    endTransaction();
    buffer = 0;
    startTransaction(BNO055_ADDRESS_ACCELGYRO);
    MEReadReg(BNO055_ACC_ID, &buffer, 1);
    if (buffer > 0)
    {
        data++;
    }
    endTransaction();
    startTransaction(BNO055_ADDRESS_ACCELGYRO);
    MEReadReg(BNO055_MAG_ID, &buffer, 1);
    if (buffer > 0)
    {
        data++;
    }
    endTransaction();



    return data;
}

bool startTransaction(uint8_t device)
{
    // Select slave
    return MESelectSlave(device);
}

void endTransaction()
{
    MEDeselectSlave();
}


void _InitialBoschSetup()
{
    // CONFIGMODE  === 0x01
    // Select BNO055 config mode

       uint8_t data;
       startTransaction(BNO055_ADDRESS_ACCELGYRO);

       data = 0x00;
       MEWriteReg(BNO055_OPR_MODE, &data, 1);  //setup config mode      0x00
       delay_ms(2000);

       data = 0x01;
       MEWriteReg(BNO055_PAGE_ID, &data, 1);     // Select page 1 to configure sensors

       data = 0x0E;
       MEWriteReg(BNO055_ACC_CONFIG, &data, 1);     //accel config

       data = 0x13;
       MEWriteReg(BNO055_GYRO_CONFIG_0, &data, 1);     //gyro config zero

       data = 0x00;
       MEWriteReg(BNO055_GYRO_CONFIG_1, &data, 1);     //gyro config one

       data = 0x0F;
       MEWriteReg(BNO055_MAG_CONFIG, &data, 1);     //mag config

       data = 0x00;
       MEWriteReg(BNO055_PAGE_ID, &data, 1);     // Select page 1 to configure sensors

//       delay_ms(200);

       data = 0x01;
       MEWriteReg(BNO055_TEMP_SOURCE, &data, 1);  //setup read mode      0x00

       data = 0x01;
       MEWriteReg(BNO055_UNIT_SEL, &data, 1);    // Select BNO055 sensor units (temperature in degrees C, rate in dps, accel in mg)

       data = 0x00;
       MEWriteReg(BNO055_PWR_MODE, &data, 1);  // power mode 0x00 is normal... i think


       data = 0x07;
       MEWriteReg(BNO055_OPR_MODE, &data, 1);  // this should turn all three on



       endTransaction();
}



void _setUpBosch()
{
    uint8_t data;

    startTransaction(BNO055_ADDRESS_ACCELGYRO);

    data = 0x07;
    MEWriteReg(BNO055_OPR_MODE, &data, 1);  // this should turn all three on


    delay_ms(200);
    
    endTransaction();

    InitialBoschSetup();


}

void _readAccelerometerData(uint8_t *destination)
{
    startTransaction(BNO055_ADDRESS_ACCELGYRO);
    uint8_t buffer[6] = { 0 };


    MEReadReg(BNO055_ACC_DATA_X_LSB, &buffer[0], 1);
    MEReadReg(BNO055_ACC_DATA_X_MSB, &buffer[1], 1);
    MEReadReg(BNO055_ACC_DATA_Y_LSB, &buffer[2], 1);
    MEReadReg(BNO055_ACC_DATA_Y_MSB, &buffer[3], 1);
    MEReadReg(BNO055_ACC_DATA_Z_LSB, &buffer[4], 1);
    MEReadReg(BNO055_ACC_DATA_Z_MSB, &buffer[5], 1);

    memcpy(destination, &buffer[0], 6);
    endTransaction();
}


void _readGyroscopeData(uint8_t *destination)
{
    startTransaction(BNO055_ADDRESS_ACCELGYRO);
    uint8_t buffer[6] = { 0 };
    MEReadReg(BNO055_GYR_DATA_X_LSB, &buffer[0], 1);
    MEReadReg(BNO055_GYR_DATA_X_MSB, &buffer[1], 1);
    MEReadReg(BNO055_GYR_DATA_Y_LSB, &buffer[2], 1);
    MEReadReg(BNO055_GYR_DATA_Y_MSB, &buffer[3], 1);
    MEReadReg(BNO055_GYR_DATA_Z_LSB, &buffer[4], 1);
    MEReadReg(BNO055_GYR_DATA_Z_MSB, &buffer[5], 1);
    memcpy(destination, &buffer[0], 6);
    endTransaction();
}


void _readMagData(uint8_t *destination)
{
    startTransaction(BNO055_ADDRESS_MAG);
    uint8_t buffer[6] = { 0 };
    MEReadReg(BNO055_MAG_DATA_X_LSB, &buffer[0], 1);
    MEReadReg(BNO055_MAG_DATA_X_MSB, &buffer[1], 1);
    MEReadReg(BNO055_MAG_DATA_Y_LSB, &buffer[2], 1);
    MEReadReg(BNO055_MAG_DATA_Y_MSB, &buffer[3], 1);
    MEReadReg(BNO055_MAG_DATA_Z_LSB, &buffer[4], 1);
    MEReadReg(BNO055_MAG_DATA_Z_MSB, &buffer[5], 1);
    memcpy(destination, &buffer[0], 6);
    endTransaction();
}


void _readInterruptData(uint8_t *destination)
{
    startTransaction(BNO055_ADDRESS_ACCELGYRO);
    uint8_t buffer[1] = { 0 };
    MEReadReg(BNO055_REGISTER_STATUS_REG, &buffer[0], 1);
    memcpy(destination, &buffer[0], 1);
    endTransaction();
}


void _readMagOnData(uint8_t *destination)
{
    startTransaction(BNO055_ADDRESS_MAG);
    uint8_t buffer[1] = { 0 };
    MEReadReg(BNO055_REGISTER_CTRL_REG2_M, &buffer[0], 1);
    memcpy(destination, &buffer[0], 1);
    endTransaction();
}


