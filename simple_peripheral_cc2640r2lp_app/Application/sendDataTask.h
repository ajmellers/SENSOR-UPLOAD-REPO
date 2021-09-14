
#define SBP_SEND_TASK_PRIORITY                2

#ifndef SBP_SEND_TASK_STACK_SIZE
#define SBP_SEND_TASK_STACK_SIZE              400
#endif

#define PAGES_IN_SECTOR                       16

extern void createSendDataTask(void);
