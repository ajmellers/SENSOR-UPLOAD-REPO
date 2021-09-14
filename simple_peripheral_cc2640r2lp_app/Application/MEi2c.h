/*
 * Copyright (c) 2018 MedEngine GmbH
 * All rights reserved.
 * Author: Christian Mellor (christian.mellor@medengine.co)
 *
 * I2C Wrapper functions to prevent DA
 *
 */
#include "stdbool.h"
#include "stdint.h"
/*********************************************************************
 * CONSTANTS
 */
#define I2C_INTERFACE_0     0
#define I2C_INTERFACE_1     1
#define I2C_INTERFACE_NONE  -1
extern bool MEinitI2c();
extern bool MESelectSlave(uint8_t address);
extern void MEDeselectSlave();
extern bool MEI2cWriteRead(uint8_t *wdata, uint8_t wlen, uint8_t *rdata, uint8_t rlen);
extern bool MEReadReg(uint8_t addr, uint8_t *pBuf, uint8_t nBytes);
extern bool MEWriteReg(uint8_t addr, uint8_t *pBuf, uint8_t nBytes);
