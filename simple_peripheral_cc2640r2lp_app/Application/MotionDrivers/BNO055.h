/*
 * Copyright (c) 2018 MedEngine GmbH
 * All rights reserved.
 *
 * BNO055 Header
 *
 * Define the required motion chip in compiler defines
 *
 *
 */
#include "motionDriver.h"

#define BNO055_ADDRESS_ACCELGYRO  0x28 //14 USED TO WORK
#define BNO055_ADDRESS_MAG        0x28   //1c USED TO WORK // these need changing before the  address will work
#define BNO055_XG_ID              0b01101000
#define BNO055_MAG_ID             0b00111101


// BNO055 Page 0
#define BNO055_CHIP_ID          0x00    // should be 0xA0
#define BNO055_ACC_ID           0x01    // should be 0xFB
#define BNO055_MAG_ID           0x02    // should be 0x32
#define BNO055_GYRO_ID          0x03    // should be 0x0F
#define BNO055_SW_REV_ID_LSB    0x04
#define BNO055_SW_REV_ID_MSB    0x05
#define BNO055_BL_REV_ID        0x06
#define BNO055_PAGE_ID          0x07
#define BNO055_ACC_DATA_X_LSB   0x08
#define BNO055_ACC_DATA_X_MSB   0x09
#define BNO055_ACC_DATA_Y_LSB   0x0A
#define BNO055_ACC_DATA_Y_MSB   0x0B
#define BNO055_ACC_DATA_Z_LSB   0x0C
#define BNO055_ACC_DATA_Z_MSB   0x0D
#define BNO055_MAG_DATA_X_LSB   0x0E
#define BNO055_MAG_DATA_X_MSB   0x0F
#define BNO055_MAG_DATA_Y_LSB   0x10
#define BNO055_MAG_DATA_Y_MSB   0x11
#define BNO055_MAG_DATA_Z_LSB   0x12
#define BNO055_MAG_DATA_Z_MSB   0x13
#define BNO055_GYR_DATA_X_LSB   0x14
#define BNO055_GYR_DATA_X_MSB   0x15
#define BNO055_GYR_DATA_Y_LSB   0x16
#define BNO055_GYR_DATA_Y_MSB   0x17
#define BNO055_GYR_DATA_Z_LSB   0x18
#define BNO055_GYR_DATA_Z_MSB   0x19
#define BNO055_EUL_HEADING_LSB  0x1A
#define BNO055_EUL_HEADING_MSB  0x1B
#define BNO055_EUL_ROLL_LSB     0x1C
#define BNO055_EUL_ROLL_MSB     0x1D
#define BNO055_EUL_PITCH_LSB    0x1E
#define BNO055_EUL_PITCH_MSB    0x1F
#define BNO055_QUA_DATA_W_LSB   0x20
#define BNO055_QUA_DATA_W_MSB   0x21
#define BNO055_QUA_DATA_X_LSB   0x22
#define BNO055_QUA_DATA_X_MSB   0x23
#define BNO055_QUA_DATA_Y_LSB   0x24
#define BNO055_QUA_DATA_Y_MSB   0x25
#define BNO055_QUA_DATA_Z_LSB   0x26
#define BNO055_QUA_DATA_Z_MSB   0x27
#define BNO055_LIA_DATA_X_LSB   0x28
#define BNO055_LIA_DATA_X_MSB   0x29
#define BNO055_LIA_DATA_Y_LSB   0x2A
#define BNO055_LIA_DATA_Y_MSB   0x2B
#define BNO055_LIA_DATA_Z_LSB   0x2C
#define BNO055_LIA_DATA_Z_MSB   0x2D
#define BNO055_GRV_DATA_X_LSB   0x2E
#define BNO055_GRV_DATA_X_MSB   0x2F
#define BNO055_GRV_DATA_Y_LSB   0x30
#define BNO055_GRV_DATA_Y_MSB   0x31
#define BNO055_GRV_DATA_Z_LSB   0x32
#define BNO055_GRV_DATA_Z_MSB   0x33
#define BNO055_TEMP             0x34
#define BNO055_CALIB_STAT       0x35
#define BNO055_ST_RESULT        0x36
#define BNO055_INT_STATUS       0x37
#define BNO055_SYS_CLK_STATUS   0x38
#define BNO055_SYS_STATUS       0x39
#define BNO055_SYS_ERR          0x3A
#define BNO055_UNIT_SEL         0x3B
#define BNO055_OPR_MODE         0x3D
#define BNO055_PWR_MODE         0x3E
#define BNO055_SYS_TRIGGER      0x3F
#define BNO055_TEMP_SOURCE      0x40
#define BNO055_AXIS_MAP_CONFIG  0x41
#define BNO055_AXIS_MAP_SIGN    0x42
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_MAG_OFFSET_X_MSB 0x5C
#define BNO055_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_MAG_OFFSET_Z_MSB 0x60
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_GYR_OFFSET_X_MSB 0x62
#define BNO055_GYR_OFFSET_Y_LSB 0x63
#define BNO055_GYR_OFFSET_Y_MSB 0x64
#define BNO055_GYR_OFFSET_Z_LSB 0x65
#define BNO055_GYR_OFFSET_Z_MSB 0x66
#define BNO055_ACC_RADIUS_LSB   0x67
#define BNO055_ACC_RADIUS_MSB   0x68
#define BNO055_MAG_RADIUS_LSB   0x69
#define BNO055_MAG_RADIUS_MSB   0x6A
//
// BNO055 Page 1
#define BNO055_PAGE_ID          0x07
#define BNO055_ACC_CONFIG       0x08
#define BNO055_MAG_CONFIG       0x09
#define BNO055_GYRO_CONFIG_0    0x0A
#define BNO055_GYRO_CONFIG_1    0x0B
#define BNO055_ACC_SLEEP_CONFIG 0x0C
#define BNO055_GYR_SLEEP_CONFIG 0x0D
#define BNO055_INT_MSK          0x0F
#define BNO055_INT_EN           0x10
#define BNO055_ACC_AM_THRES     0x11
#define BNO055_ACC_INT_SETTINGS 0x12
#define BNO055_ACC_HG_DURATION  0x13
#define BNO055_ACC_HG_THRESH    0x14
#define BNO055_ACC_NM_THRESH    0x15
#define BNO055_ACC_NM_SET       0x16
#define BNO055_GYR_INT_SETTINGS 0x17
#define BNO055_GYR_HR_X_SET     0x18
#define BNO055_GYR_DUR_X        0x19
#define BNO055_GYR_HR_Y_SET     0x1A
#define BNO055_GYR_DUR_Y        0x1B
#define BNO055_GYR_HR_Z_SET     0x1C
#define BNO055_GYR_DUR_Z        0x1D
#define BNO055_GYR_AM_THRESH    0x1E
#define BNO055_GYR_AM_SET       0x1F

/////////////////////////////////////////////////////////////////


// Linear Acceleration: mg per LSB
#define BNO055_ACCEL_MG_LSB_2G (0.061F)
#define BNO055_ACCEL_MG_LSB_4G (0.122F)
#define BNO055_ACCEL_MG_LSB_8G (0.244F)
#define BNO055_ACCEL_MG_LSB_16G (0.732F)

// Magnetic Field Strength: gauss range
#define BNO055_MAG_MGAUSS_4GAUSS      (0.14F)
#define BNO055_MAG_MGAUSS_8GAUSS      (0.29F)
#define BNO055_MAG_MGAUSS_12GAUSS     (0.43F)
#define BNO055_MAG_MGAUSS_16GAUSS     (0.58F)

// Angular Rate: dps per LSB
#define BNO055_GYRO_DPS_DIGIT_245DPS      (0.00875F)
#define BNO055_GYRO_DPS_DIGIT_500DPS      (0.01750F)
#define BNO055_GYRO_DPS_DIGIT_2000DPS     (0.07000F)

// Temperature: LSB per degree celsius
#define BNO055_TEMP_LSB_DEGREE_CELSIUS    (8)

/*
 * Accelerometer & Gyroscope Registers
 */
typedef enum
{
    ACT_THS = 0x04,
    INT_GEN_CFG_XL  = 0x06,
    INT_GEN_THS_X_XL = 0x07,
    INT_GEN_THS_Y_XL = 0x08,
    INT_GEN_THS_Z_XL = 0x09,
    INT_GEN_DUR_XL = 0X0A,
    BNO055_REGISTER_WHO_AM_I_XG = 0x01,
    BNO055_REGISTER_CTRL_REG1_G = 0x10,
    BNO055_REGISTER_CTRL_REG2_G = 0x11,
    BNO055_REGISTER_CTRL_REG3_G = 0x12,
    BNO055_REGISTER_TEMP_OUT_L = 0x15,
    BNO055_REGISTER_TEMP_OUT_H = 0x16,
    BNO055_REGISTER_STATUS_REG = 0x17,
    BNO055_REGISTER_OUT_X_L_G = 0x18,
    BNO055_REGISTER_OUT_X_H_G = 0x19,
    BNO055_REGISTER_OUT_Y_L_G = 0x1A,
    BNO055_REGISTER_OUT_Y_H_G = 0x1B,
    BNO055_REGISTER_OUT_Z_L_G = 0x1C,
    BNO055_REGISTER_OUT_Z_H_G = 0x1D,
    BNO055_REGISTER_CTRL_REG4 = 0x1E,
    BNO055_REGISTER_CTRL_REG5_XL = 0x1F,
    BNO055_REGISTER_CTRL_REG6_XL = 0x20,
    BNO055_REGISTER_CTRL_REG7_XL = 0x21,
    BNO055_REGISTER_CTRL_REG8 = 0x22,
    BNO055_REGISTER_CTRL_REG9 = 0x23,
    BNO055_REGISTER_CTRL_REG10 = 0x24,
    BNO055_REGISTER_OUT_X_L_XL = 0x28,
    BNO055_REGISTER_OUT_X_H_XL = 0x29,
    BNO055_REGISTER_OUT_Y_L_XL = 0x2A,
    BNO055_REGISTER_OUT_Y_H_XL = 0x2B,
    BNO055_REGISTER_OUT_Z_L_XL = 0x2C,
    BNO055_REGISTER_OUT_Z_H_XL = 0x2D,
    BNO055XG_FIFO_CTRL         = 0X2E,
    BNO055XG_FIFO_SRC         = 0x2F
} BNO055AccGyroRegisters_t;

typedef struct {
 uint8_t int1_drdy_xl      : 1;
 uint8_t int1_drdy_g       : 1;
 uint8_t int1_boot         : 1;
 uint8_t int1_fth          : 1;
 uint8_t int1_ovr          : 1;
 uint8_t int1_fss5         : 1;
 uint8_t int1_ig_xl        : 1;
 uint8_t int1_ig_g         : 1;
} BNO055_pin_int1_route_t;

#define BNO055_INT1_CTRL        0x0C
typedef struct {
 uint8_t int1_drdy_xl             : 1;
 uint8_t int1_drdy_g              : 1;
 uint8_t int1_boot                : 1;
 uint8_t int1_fth                 : 1;
 uint8_t int1_ovr                 : 1;
 uint8_t int1_fss5                : 1;
 uint8_t int1_ig_xl               : 1;
 uint8_t int1_ig_g                : 1;
} BNO055_int1_ctrl_t;

int32_t BNO055_pin_int1_route_set(
                                  BNO055_pin_int1_route_t val);

typedef union {
    BNO055_int1_ctrl_t              int1_ctrl;
}BNO055_reg_t;


/*
 * Compass Registers
 */
typedef enum
{
    BNO055M_OFFSET_X_REG_L_M =  0x05,
    BNO055M_OFFSET_X_REG_H_M  = 0x06,
    BNO055M_OFFSET_Y_REG_L_M  = 0x07,
    BNO055M_OFFSET_Y_REG_H_M  = 0x08,
    BNO055M_OFFSET_Z_REG_L_M  = 0x09,
    BNO055M_OFFSET_Z_REG_H_M  = 0x0A,
    BNO055_REGISTER_WHO_AM_I_M = 0x00,
    BNO055_REGISTER_CTRL_REG1_M = 0x20,
    BNO055_REGISTER_CTRL_REG2_M = 0x21,
    BNO055_REGISTER_CTRL_REG3_M = 0x22,
    BNO055_REGISTER_CTRL_REG4_M = 0x23,
    BNO055_REGISTER_CTRL_REG5_M = 0x24,
    BNO055_REGISTER_STATUS_REG_M = 0x27,
    BNO055_REGISTER_OUT_X_L_M = 0x28,
    BNO055_REGISTER_OUT_X_H_M = 0x29,
    BNO055_REGISTER_OUT_Y_L_M = 0x2A,
    BNO055_REGISTER_OUT_Y_H_M = 0x2B,
    BNO055_REGISTER_OUT_Z_L_M = 0x2C,
    BNO055_REGISTER_OUT_Z_H_M = 0x2D,
    BNO055_REGISTER_CFG_M = 0x30,
    BNO055_REGISTER_INT_SRC_M = 0x31,
} BNO055MagRegisters_t;
/*
 * Accelerometer range settings
 */
typedef enum
{
    BNO055_ACCELRANGE_2G = (0b00 << 3),
    BNO055_ACCELRANGE_16G = (0b01 << 3),
    BNO055_ACCELRANGE_4G = (0b10 << 3),
    BNO055_ACCELRANGE_8G = (0b11 << 3),
} BNO055AccelRange_t;
/*
 * Accel. data rate
 */
typedef enum
{
    BNO055_ACCELDATARATE_POWERDOWN = (0b0000 << 4),
    BNO055_ACCELDATARATE_3_125HZ = (0b0001 << 4),
    BNO055_ACCELDATARATE_6_25HZ = (0b0010 << 4),
    BNO055_ACCELDATARATE_12_5HZ = (0b0011 << 4),
    BNO055_ACCELDATARATE_25HZ = (0b0100 << 4),
    BNO055_ACCELDATARATE_50HZ = (0b0101 << 4),
    BNO055_ACCELDATARATE_100HZ = (0b0110 << 4),
    BNO055_ACCELDATARATE_200HZ = (0b0111 << 4),
    BNO055_ACCELDATARATE_400HZ = (0b1000 << 4),
    BNO055_ACCELDATARATE_800HZ = (0b1001 << 4),
    BNO055_ACCELDATARATE_1600HZ = (0b1010 << 4)
} lm9ds1AccelDataRate_t;

/*
 * Compass gain
 */
typedef enum
{
    BNO055_MAGGAIN_4GAUSS = (0b00 << 5),  // +/- 4 gauss
    BNO055_MAGGAIN_8GAUSS = (0b01 << 5),  // +/- 8 gauss
    BNO055_MAGGAIN_12GAUSS = (0b10 << 5),  // +/- 12 gauss
    BNO055_MAGGAIN_16GAUSS = (0b11 << 5)   // +/- 16 gauss
} BNO055MagGain_t;
/*
 * Compass data rates
 */
typedef enum
{
    BNO055_MAGDATARATE_3_125HZ = (0b000 << 2),
    BNO055_MAGDATARATE_6_25HZ = (0b001 << 2),
    BNO055_MAGDATARATE_12_5HZ = (0b010 << 2),
    BNO055_MAGDATARATE_25HZ = (0b011 << 2),
    BNO055_MAGDATARATE_50HZ = (0b100 << 2),
    BNO055_MAGDATARATE_100HZ = (0b101 << 2)
} BNO055MagDataRate_t;
/*
 * Gyroscope scale
 */
typedef enum
{
    BNO055_GYROSCALE_245DPS = (0b00 << 3), // +/- 245 degrees per second rotation
    BNO055_GYROSCALE_500DPS = (0b01 << 3), // +/- 500 degrees per second rotation
    BNO055_GYROSCALE_2000DPS = (0b11 << 3) // +/- 2000 degrees per second rotation
} BNO055GyroScale_t;

extern bool init();

extern bool isWomEnabled();

extern uint8_t _whoAmI();
extern bool startTransaction(uint8_t device);
extern void endTransaction();
extern void _InitialBoschSetup();
extern void _readInterruptData();
extern void _setUpBosch();
extern void _readAccelerometerData(uint8_t *data);
extern void _readGyroscopeData(uint8_t *data);
extern void _readMagData(uint8_t *data);

extern void _readMagOnData(uint8_t *data);
