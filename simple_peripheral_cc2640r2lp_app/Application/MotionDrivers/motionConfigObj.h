/*
 * Copyright (c) 2018 MedEngine GmbH
 * All rights reserved.
 * Author: Christian Mellor (christian.mellor@medengine.co)
 *
 * Motion Config Object
 *
 * A struct to hold motion configuration data
 *
 *
 */
#include <stdint.h>
#ifndef ME_MOTION_CONFIG_OBJ
#define ME_MOTION_CONFIG_OBJ
typedef struct MEmotionConfigObj
{
    uint8_t calibration;
    uint8_t enable;
    uint8_t accel_axes;
    uint8_t gyro_axes;
    uint8_t mag_axes;
    uint8_t accel_range;
    uint8_t gyro_range;
    uint8_t mag_range;
} MEmotionConfigObj;
extern MEmotionConfigObj currentConfig;
#endif

