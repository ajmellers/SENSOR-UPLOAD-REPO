#define SAMPLING_INTERVAL                     25
#define SBP_READING_SENSOR_TASK_PRIORITY      1
#ifndef SBP_READING_SENSOR_TASK_STACK_SIZE
#define SBP_READING_SENSOR_TASK_STACK_SIZE    560
#endif
#define SAMPLE_LEN                 23
#define SAVE_DATA_EVENT     Event_Id_00
#define SAMPLES_NUMBER_IN_FRAME    10
#define INCLUDE_MEMORY_TEST 0

extern void createReadSensorDataTask(void);
