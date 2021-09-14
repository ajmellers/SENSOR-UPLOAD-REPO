#include "timeManager.h"
#include "stdlib.h"
#include "icall.h"

#include <ti/display/Display.h>
extern Display_Handle dispHandle;

#define UNIX_TO_UTC_TIME_SECONDS 946684800
#define NANOSECONDS_DIVIDER 3906250

UTCTime utcTime;
uint32 unixTime;
uint32_t fraction;
UTCTimeStruct dateTime;
TimeController timeController;
static MemoryTimestamp memoryTimestampHolder;
static Seconds_Time secondsTimeHolder;

static UTCTime getTime(){
    return osal_getClock();
}

static uint8_t getSeconds(Seconds_Time *ts){
    return Seconds_getTime(ts);
}

static uint8_t getFraction(uint32_t nanoseconds) {
    fraction = nanoseconds/NANOSECONDS_DIVIDER;
    return (uint8_t)fraction;
}

void initTimeController(){
    timeController.time = &getTime;
    timeController.seconds = &getSeconds;
    timeController.time();
}

void convertToMemoryTimestamp(Seconds_Time *ts, MemoryTimestamp *memoryTimestamp){
    memoryTimestamp->seconds = ts->secs;
    memoryTimestamp->fractions = getFraction(ts->nsecs);
}

MemoryTimestamp* getMemoryTimestamp(){
    timeController.seconds(&secondsTimeHolder);
    memoryTimestampHolder.seconds = secondsTimeHolder.secs;
    memoryTimestampHolder.fractions = (uint8_t)(secondsTimeHolder.nsecs/NANOSECONDS_DIVIDER);
    return &memoryTimestampHolder;
}

void getCurrentTime(union CurrentTime *currentTime){
    utcTime = timeController.time();
    osal_ConvertUTCTime(&dateTime, utcTime);
    Seconds_Time *secondsTimestamp = malloc(sizeof(Seconds_Time));

    timeController.seconds(secondsTimestamp);

    currentTime->exactTime.adj= 0;
    currentTime->exactTime.dayOfWeek = 0;
    currentTime->exactTime.fractionSeconds = getFraction(secondsTimestamp->nsecs);
    currentTime->exactTime.timestamp = dateTime;

    free(secondsTimestamp);
}

void updateCurrentTime(UTCTimeStruct *timestamp){
    utcTime = osal_ConvertUTCSecs(timestamp);
    unixTime = (uint32)utcTime + UNIX_TO_UTC_TIME_SECONDS;
    osal_setClock(utcTime);
    Seconds_set(unixTime);
}
