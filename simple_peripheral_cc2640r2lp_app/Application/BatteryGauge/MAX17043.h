//
// Created by Alex Mellers on 08/05/2019.
//

#include "stdbool.h"
#include "stdint.h"

#define MAX17043_VCELL    0x02 // R - 12-bit A/D measurement of battery voltage
#define MAX17043_SOC      0x04 // R - 16-bit state of charge (SOC)
#define MAX17043_MODE     0x06 // W - Sends special commands to IC
#define MAX17043_VERSION  0x00 // R - Returns IC version
#define MAX17043_CONFIG   0x0C // R/W - Battery compensation (default 0x971C)
#define MAX17043_COMMAND  0xFE // W - Sends special commands to IC

///////////////////////////////////
// MAX17043 Config Register Bits //
///////////////////////////////////
#define MAX17043_CONFIG_SLEEP     7
#define MAX17043_CONFIG_ALERT     5
#define MAX17043_CONFIG_THRESHOLD 0

/////////////////////////////////////
// MAX17043 Mode Register Commands //
/////////////////////////////////////
#define MAX17043_MODE_QUICKSTART 0x40

////////////////////////////////////////
// MAX17043 Command Register Commands //
////////////////////////////////////////
#define MAX17043_COMMAND_POR 0x54
////////////////////////////////
// MAX17043 7-Bit I2C Address //
////////////////////////////////
#define MAX17043_ADDRESS  0x36

extern bool startGaugeTransaction(uint8_t device);

extern void MAX17043Run();

extern void MAX17043Version(uint8_t *data);

extern void MAX17043StateOfCharge(uint8_t *data);
