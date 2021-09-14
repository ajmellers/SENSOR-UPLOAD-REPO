/*
 * Copyright (c) 2018 MedEngine GmbH
 * All rights reserved.
 * Author: Christian Mellor (christian.mellor@medengine.co)
 *
 * I2C Wrapper functions to prevent DA
 *
 */
#include "MEi2c.h"
#include "stdbool.h"
#include "stdint.h"
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <driverlib/prcm.h>
#include "Board.h"
#include <ti/sysbios/knl/Clock.h>
#define delay_ms(i) Task_sleep( ((i) * 1000) / Clock_tickPeriod )
#define MS_2_TICKS(ms) ( ((ms) * 1000) / Clock_tickPeriod )
/*******************************************************************************
 * LOCAL variables
 */
static volatile uint8_t slaveAddr;
static volatile uint8_t interface;
static I2C_Handle i2cHandle;
static I2C_Params i2cParams;
static Semaphore_Struct mutex;
static const I2CCC26XX_I2CPinCfg pinCfg1 = {
// Pin configuration for I2C interface 1
        .pinSDA = CC2640R2_LAUNCHXL_I2C0_SDA0, .pinSCL = CC2640R2_LAUNCHXL_I2C0_SCL0 };
/*******************************************************************************
 * CONSTANTS
 */
#define I2C_TIMEOUT 500
bool MEinitI2c()
{
    Semaphore_Params semParamsMutex;
    // Create protection semaphore to stop other services using I2C
    Semaphore_Params_init(&semParamsMutex);
    semParamsMutex.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&mutex, 1, &semParamsMutex);
    I2C_init();
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2cParams.custom = (uintptr_t) &pinCfg1;
    i2cHandle = I2C_open(Board_I2C_TMP, &i2cParams);
    // Initialise local variables
    slaveAddr = 0xFF;
    interface = I2C_INTERFACE_0;
    if (i2cHandle == NULL)
    {
        Task_exit();
    }
    delay_ms(100);
    return true;
}

bool MESelectSlave(uint8_t address)
{
    // Acquire I2C resource
    if (!Semaphore_pend(Semaphore_handle(&mutex), MS_2_TICKS(I2C_TIMEOUT)))
    {
        return false;
    }
    // Store new slave address
    slaveAddr = address;
    interface = I2C_INTERFACE_1;
    return true;
}
void MEDeselectSlave()
{
    slaveAddr = 0xFF;
    interface = I2C_INTERFACE_0;
    // Release I2C resource
    Semaphore_post(Semaphore_handle(&mutex));
}
/*******************************************************************************
 * @fn          MEI2cWriteRead
 *
 * @brief       Burst write/read from an I2C device
 *
 * @param       wdata - pointer to write data buffer
 * @param       wlen - number of bytes to write
 * @param       rdata - pointer to read data buffer
 * @param       rlen - number of bytes to read
 *
 * @return      true if success
 */
bool MEI2cWriteRead(uint8_t *wdata, uint8_t wlen, uint8_t *rdata, uint8_t rlen)
{
    I2C_Transaction masterTransaction;
    if (interface == I2C_INTERFACE_0)
    {
        return false;
    }
    masterTransaction.writeCount = wlen;
    masterTransaction.writeBuf = wdata;
    masterTransaction.readCount = rlen;
    masterTransaction.readBuf = rdata;
    masterTransaction.slaveAddress = slaveAddr;
    return I2C_transfer(i2cHandle, &masterTransaction) == TRUE;
}
// Read and write to I2C devices
static uint8_t buffer[32];
/*******************************************************************************
 * @fn          MEReadReg
 *
 * @brief       This function implements the I2C protocol to read from a sensor.
 *              The sensor must be selected before this routine is called.
 *
 * @param       addr - which register to read`
 * @param       pBuf - pointer to buffer to place data
 * @param       nBytes - number of bytes to read
 *
 * @return      TRUE if the required number of bytes are received
 ******************************************************************************/
bool MEReadReg(uint8_t addr, uint8_t *pBuf, uint8_t nBytes)
{
    return MEI2cWriteRead(&addr, 1, pBuf, nBytes);
}
/*******************************************************************************
 * @fn          MEWriteReg
 * @brief       This function implements the I2C protocol to write to a sensor.
 *              The sensor must be selected before this routine is called.
 *
 * @param       addr - which register to write
 * @param       pBuf - pointer to buffer containing data to be written
 * @param       nBytes - number of bytes to write
 *
 * @return      TRUE if successful write
 */
bool MEWriteReg(uint8_t addr, uint8_t *pBuf, uint8_t nBytes)
{
    uint8_t i;
    uint8_t *p = buffer;
    /* Copy address and data to local buffer for burst write */
    *p++ = addr;
    for (i = 0; i < nBytes; i++)
    {
        *p++ = *pBuf++;
    }
    nBytes++;
    /* Send data */
    return MEI2cWriteRead(buffer, nBytes, NULL, 0);
}
