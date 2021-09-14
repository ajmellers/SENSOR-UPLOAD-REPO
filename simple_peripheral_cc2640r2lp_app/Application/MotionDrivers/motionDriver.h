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
#include "stdbool.h"
#include "stdint.h"
#include "motionConfigObj.h"
extern bool initialise();
extern bool wakeOnMotion();
extern uint8_t whoAmI();
extern void setUpBosch();
extern void setUpMAG();
extern void magConfig();
extern void accelGyroConfig();
extern void InitialBoschSetup();
extern void readInterruptData(uint8_t *pointer);
extern void readMagOnData(uint8_t *pointer);
extern void readAccelerometerData(uint8_t *pointer);
extern void readGyroscopeData(uint8_t *pointer);
extern void readMagData(uint8_t *pointer);
extern bool arrayToConfig(uint8_t *configArray);
