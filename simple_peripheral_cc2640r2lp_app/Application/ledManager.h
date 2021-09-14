#ifndef APPLICATION_BLINKLED_H_
#define APPLICATION_BLINKLED_H_

struct LedsState{
  uint8_t whiteLed;
  uint8_t redLed;
};

extern struct LedsState ledsState = {0, 0};

extern void setRedLedPin(uint32_t value);

extern void setWhiteLedPin(uint32_t value);

extern void setLed3Pin(uint32_t value);



#endif /* APPLICATION_BLINKLED_H_ */
