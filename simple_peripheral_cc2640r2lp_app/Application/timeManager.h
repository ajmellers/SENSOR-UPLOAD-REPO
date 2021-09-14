#ifndef APPLICATION_TIMEMANAGER_H_
#define APPLICATION_TIMEMANAGER_H_

#include <ti/sysbios/hal/Seconds.h>
#include "osal.h"
#include "timeService.h"
#include "osal_clock.h"

typedef struct
{
    UTCTime(* time)();
    uint8_t(* seconds)(Seconds_Time *);
}TimeController;

#pragma pack(push, 1)
typedef struct
{
    uint32_t seconds;
    uint8_t fractions;
}MemoryTimestamp;
#pragma pack(pop)

typedef struct
{
    uint8_t adj;
    uint8_t fractionSeconds;
    uint8_t dayOfWeek;
    UTCTimeStruct timestamp;
}UTCExactTime;

union CurrentTime {
    UTCExactTime exactTime;
    uint8_t byteArrayForBLE[TIMESERVICE_DATA_LEN];
};

extern void initTimeController();

extern void convertToMemoryTimestamp(Seconds_Time *ts, MemoryTimestamp *memoryTimestamp);

extern MemoryTimestamp* getMemoryTimestamp();

extern void getCurrentTime(union CurrentTime *currentTime);

extern void updateCurrentTime(UTCTimeStruct *timestamp);

#endif /* APPLICATION_TIMEMANAGER_H_ */
