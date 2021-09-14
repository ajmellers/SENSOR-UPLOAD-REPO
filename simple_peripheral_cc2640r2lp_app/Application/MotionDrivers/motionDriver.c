/*
 * Copyright (c) 2018 MedEngine GmbH
 * All rights reserved.
 *
 * MotionDriver
 *
 * Define the required motion chip in compiler defines
 *
 *
 */
#include <MotionDrivers/BNO055.h>
#include "stdbool.h"
#include "stdint.h"
#include "motionConfigObj.h"
#include "motionDriver.h"

bool isInitialised = false;
/*
 * currentConfig is defined externally
 */
MEmotionConfigObj currentConfig;
bool initialise()
{
    return init();
}
bool wakeOnMotion()
{
    return isWomEnabled();
}
uint8_t whoAmI()
{
    return _whoAmI();
}
void setUpBosch()
{
    return _setUpBosch();
}
void InitialBoschSetup()
{
    return _InitialBoschSetup();
}
void readAccelerometerData(uint8_t *pointer)
{
    return _readAccelerometerData(pointer);
}
void readGyroscopeData(uint8_t *pointer)
{
    return _readGyroscopeData(pointer);
}
void readMagData(uint8_t *pointer)
{
    return _readMagData(pointer);
}
void  readInterruptData(uint8_t *pointer)
{
    return _readInterruptData(pointer);
}
void readMagOnData(uint8_t *pointer)
{
    return _readMagOnData(pointer);
}
bool arrayToConfig(uint8_t *configArray)
{
    // todo Validate the input from the array
    currentConfig.calibration = configArray[0];
    currentConfig.enable = configArray[1];
    currentConfig.accel_axes = configArray[2];
    currentConfig.gyro_axes = configArray[3];
    currentConfig.mag_axes = configArray[4];
    currentConfig.accel_range = configArray[5];
    currentConfig.gyro_range = configArray[6];
    currentConfig.mag_range = configArray[7];
    return true;
}
