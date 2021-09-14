#include <string.h>
#include "MESpi.h"
#include "timeManager.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/display/Display.h>
#include "util.h"
#include "icall.h"
#include "MotionDrivers/motionDriver.h"
#include "simple_peripheral.h"
#if defined( USE_FPGA ) || defined( DEBUG_SW_TRACE )
#include <driverlib/ioc.h>
#endif // USE_FPGA | DEBUG_SW_TRACE
#include <readSensorDataTask.h>

Task_Struct sbpReadSensorDataTask;
Char sbpReadSensorDataTaskStack[SBP_READING_SENSOR_TASK_STACK_SIZE];

static void readSensorDataTask(UArg a0, UArg a1);
static void flyttaReadMotionData(uint8_t *dataFrame);
static void initMotionDriver(void);
static void addTimestamp(uint8_t *pointer);
static void storeMotionData(void);
static void fillDataFrame(uint8_t *data);
static void sensorEventClockHandler(UArg arg);

extern Display_Handle dispHandle;
static uint8_t sampleCounter = 0;
static ICall_SyncHandle sensorEvent;
static ICall_EntityID sensorEntity;
static Clock_Struct saveDataClock;
extern const Semaphore_Handle spiMutex;

/*********************************************************************
* @fn      createReadSensorDataTask
*
* @brief   Task creation function.
*
* @param   None.
*
* @return  None.
*/
void createReadSensorDataTask(void) {
    Task_Params readSensorDataTaskParams;

    Task_Params_init(&readSensorDataTaskParams);
    readSensorDataTaskParams.stack = sbpReadSensorDataTaskStack;
    readSensorDataTaskParams.stackSize = SBP_READING_SENSOR_TASK_STACK_SIZE;
    readSensorDataTaskParams.priority = SBP_READING_SENSOR_TASK_PRIORITY;

    Task_construct(&sbpReadSensorDataTask, readSensorDataTask, &readSensorDataTaskParams, NULL);
}

/*********************************************************************
 * @fn      readSensorDataTask
 *
 * @brief   Application task entry point.
 *
 * @param   arg0, arg1 - not used.
 *
 * @return  None.
 */
static void readSensorDataTask(UArg arg0, UArg arg1)
{
    /* Local variables. Variables here go onto task stack!! */

    /* Run one-time code when task starts */
    ICall_registerApp(&sensorEntity, &sensorEvent);

//    #ifdef INCLUDE_MEMORY_TEST
//    initSPI();
//    memoryTestCodes testCode = memoryTest();
//    closeSPI();
//    Display_print1(dispHandle, 14, 0, "Result of the memory test: %d", testCode);
//    #endif

    initTimeController();

    initMotionDriver();

    initSPI();
//    Display_print0(dispHandle, 6, 0, "[readSensorDataTask] Getting the memory pointers... ");
//    getMemoryPointers();
//    Display_print0(dispHandle, 6, 0, "[readSensorDataTask] Memory pointers set! ");

    Display_print0(dispHandle, 8, 0, "Starting readSensorDataTask");
    Util_constructClock(&saveDataClock, sensorEventClockHandler, SAMPLING_INTERVAL, 0, true, SAVE_DATA_EVENT);

    uint32_t events;
    uint8_t counter = 0;
    while(spiMutex == NULL) {
        // waiting for the semaphore to be initialized
        Display_print0(dispHandle, 14, 0, "[error: readSensorDataTask] Mutex uninitialized!");
    }
    Semaphore_post(spiMutex);

    Display_print0(dispHandle, 14, 0, "Mutex initialized!");
    while (1) /* Run loop forever (unless terminated) */
    {
        events = Event_pend(sensorEvent, Event_Id_NONE, SAVE_DATA_EVENT, ICALL_TIMEOUT_FOREVER);

        if (events & SAVE_DATA_EVENT) {
            if (getStorageStatus() != 0xFF) {
                storeMotionData();
            }
            else {
                Display_print1(dispHandle, 14, 0, "[error: readSensorDataTask] Storage full!, counter: %d", counter);
            }
            Util_startClock(&saveDataClock);
        }
        counter++;
    }
}


/*********************************************************************
 * @fn      initMotionDriver
 *
 * @brief   init motion driver.
 *
 * @return  none
 */
static void initMotionDriver(void) {
    initialise();
    setUpBosch();
}

/*********************************************************************
 * @fn      storeMotionData
 *
 * @brief   Store motion data.
 *
 * @return  none
 */
static void storeMotionData(void) {
    uint8_t *buffer = ICall_malloc(SAMPLE_LEN*sizeof(uint8_t));
    flyttaReadMotionData(buffer);
    fillDataFrame(buffer);
    ICall_free(buffer);
}

/*********************************************************************
 * @fn      syncEventClockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void sensorEventClockHandler(UArg arg) {
    // Wake up the application.
    Event_post(sensorEvent, arg);
}


/*********************************************************************
 * @fn      fillDataFrame
 *
 * @brief   fill data frame.
 *
 * @return  none
 */
static void fillDataFrame(uint8_t *data) {
    if(!(data[18]==0 && data[19]==0 && data[20]==0 && data[21]==0 && data[22]==0) && !(data[18]==0xFF && data[19]==0xFF && data[20]==0xFF && data[21]==0xFF && data[22]==0xFF )) {
        memcpy(memManager.data_to_save+(SAMPLE_LEN*sampleCounter), data,  SAMPLE_LEN);
        sampleCounter++;
        if (sampleCounter == SAMPLES_NUMBER_IN_FRAME) {
            Semaphore_pend(spiMutex, ICALL_TIMEOUT_FOREVER);
            while(!storeAndVerify()) {

            }
            Semaphore_post(spiMutex);
            sampleCounter = 0;
        }
    }
}

/*********************************************************************
 * @fn      flyttaReadMotionData
 *
 * @brief   Read motion data.
 *
 * @return  none
 */
static void flyttaReadMotionData(uint8_t *buffer) {
    readAccelerometerData(&buffer[0]);
    readGyroscopeData(&buffer[6]);
    readMagData(&buffer[12]);
    addTimestamp(&buffer[18]);
//    Display_print5(dispHandle, 14, 0, "data0: %d, %d, %d, %d, %d", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
//    Display_print5(dispHandle, 15, 0, "data1: %d, %d, %d, %d, %d", buffer[6], buffer[7], buffer[8], buffer[9], buffer[10]);
//    Display_print5(dispHandle, 16, 0, "data2: %d, %d, %d, %d, %d", buffer[12], buffer[13], buffer[14], buffer[15], buffer[16]);
    Display_print5(dispHandle, 14, 0, "data3: %d, %d, %d, %d, %d", buffer[18], buffer[19], buffer[20], buffer[21], buffer[22]);
}


/*********************************************************************
 * @fn      addTimestamp
 *
 * @brief   add timestamp to data motion data.
 *
 * @param   pointer to buffer.
 *
 * @return  none
 */
static void addTimestamp(uint8_t *pointer) {
    MemoryTimestamp *timestamp;
    timestamp = getMemoryTimestamp();
    memcpy(pointer, timestamp, sizeof(MemoryTimestamp));
}

