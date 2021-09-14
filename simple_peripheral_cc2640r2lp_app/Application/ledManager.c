
#include "board.h"

void setRedLedPin(uint32_t value);

void setWhiteLedPin(uint32_t value);

void setLed3Pin(uint32_t value);

static PIN_Handle setPin = NULL;


void setRedLedPin(uint32_t value) {
    PIN_setOutputValue(setPin, CC2640R2_LAUNCHXL_PIN_1LED, value);
}

void setWhiteLedPin(uint32_t value) {
//    PIN_setOutputValue(setPin, CC2640R2_LAUNCHXL_PIN_2LED, value);
}

void setLed3Pin(uint32_t value) {
    PIN_setOutputValue(setPin, CC2640R2_LAUNCHXL_PIN_3LED, value);
}
