#include <xdc/cfg/global.h>
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/display/Display.h>
#include <ti/sysbios/knl/Queue.h>
#include <icall.h>
#include "util.h"
#include "MESpi.h"
#include "simple_peripheral.h"
#include "motionService.h"
#include "sendDataTask.h"
#include "MotionDrivers/motionDriver.h"
#if defined( USE_FPGA ) || defined( DEBUG_SW_TRACE )
#include <driverlib/ioc.h>
#endif // USE_FPGA | DEBUG_SW_TRACE

static ICall_EntityID dataEntity;
static ICall_SyncHandle dataEvent;
static uint8_t currentAddress[ADDRESS_LENGTH] = {0};

extern Display_Handle dispHandle;

extern const Semaphore_Handle ackSemaphore;
extern const Semaphore_Handle spiMutex;
Task_Struct sbpSendDataTask;
Char sbpSendDataTaskStack[SBP_SEND_TASK_STACK_SIZE];

static void sendDataTask(UArg a0, UArg a1);
static void newDataEnqueue(uint8_t *data);
static bool processDataFromMemory();


/*********************************************************************
* @fn      createSendDataTask
*
* @brief   Task creation function.
*
* @param   None.
*
* @return  None.
*/
void createSendDataTask(void) {
    Task_Params sendDataTaskParams;

    Task_Params_init(&sendDataTaskParams);
    sendDataTaskParams.stack = sbpSendDataTaskStack;
    sendDataTaskParams.stackSize = SBP_SEND_TASK_STACK_SIZE;
    sendDataTaskParams.priority = SBP_SEND_TASK_PRIORITY;

    Task_construct(&sbpSendDataTask, sendDataTask, &sendDataTaskParams, NULL);
}

/*********************************************************************
 * @fn      sendDataTask
 *
 * @brief   Application task entry point.
 *
 * @param   arg0, arg1 - not used.
 *
 * @return  None.
 */
static void sendDataTask(UArg arg0, UArg arg1)
{
    /* Local variables. Variables here go onto task stack!! */

    /* Run one-time code when task starts */
    ICall_registerApp(&dataEntity, &dataEvent);

    uint8_t counter = 0;
    while(ackSemaphore == NULL || spiMutex == NULL) {
        // waiting for the semaphore to be initialized
        Display_print0(dispHandle, 14, 0, "[error: sendDataTask] Semaphore uninitialized!");
    }
    Semaphore_post(ackSemaphore); //to trigger the first data read
    while (1) /* Run loop forever (unless terminated) */
    {
        Display_print1(dispHandle, 12, 0, "Pending for semaphore, counter: %d", counter);
        Semaphore_pend(ackSemaphore, ICALL_TIMEOUT_FOREVER);
        uint16_t compare = compareAddresses();
        while(compare < 17) {
            compare = compareAddresses();
            Display_print1(dispHandle, 11, 0, "Difference: %u", compare);
            Task_sleep(500);
        }
        Display_print1(dispHandle, 12, 0, "Semaphore received, counter: %d", counter);

        if (counter == 0) {
            memcpy(currentAddress, memManager.first_page_addr, ADDRESS_LENGTH);
        }

        Display_print1(dispHandle, 12, 0, "Processing memory data, counter: %d", counter);
        while(!processDataFromMemory()) {
            Task_sleep(1000);
        }
        Display_print1(dispHandle, 12, 0, "Processed memory data, counter: %d", counter);
        counter++;
        if (counter == PAGES_IN_SECTOR) {
            Display_print1(dispHandle, 13, 30, "Erasing data, counter: %d", counter);
            Semaphore_pend(spiMutex, ICALL_TIMEOUT_FOREVER);
            while(!sectorErase(memManager.first_page_addr)){

            }
            Semaphore_post(spiMutex);
            counter = 0;
        } else {
            Display_print1(dispHandle, 12, 0, "Incrementing address, counter: %d", counter);
            incrementReadAddress(currentAddress);
            Display_print1(dispHandle, 12, 0, "Incremented address, counter: %d", counter);
        }
    }
//        setMemoryState();

}

/*********************************************************************
 * @fn      processDataFromMemory
 *
 * @brief   Read data from memory and put it into the queue.
 *
 * @return  none
 */
static bool processDataFromMemory() {
    if(memcmp(memManager.first_page_addr, memManager.last_page_addr, 3)) {
        Semaphore_pend(spiMutex, ICALL_TIMEOUT_FOREVER);
        if(readFromMemory(currentAddress)) {
            Display_print0(dispHandle, 12, 0, "New data enqueued");
    //        if(!((memManager.data_to_read[18]==0 && memManager.data_to_read[19]==0 && memManager.data_to_read[20]==0 && memManager.data_to_read[21]==0 && memManager.data_to_read[22]==0) || (memManager.data_to_read[18]==0xFF && memManager.data_to_read[19]==0xFF && memManager.data_to_read[20]==0xFF && memManager.data_to_read[21]==0xFF && memManager.data_to_read[22]==0xFF ))){
                newDataEnqueue(memManager.data_to_read);
    //        }
        }
        Semaphore_post(spiMutex);
        return true;
    } else {
        return false;
    }
}

/*********************************************************************
 * @fn      newDataEnqueue
 *
 * @brief   Process a new data to send.
 *
 * @param   data - data to send.
 *
 * @return  none
 */
static void newDataEnqueue(uint8_t *data) {
    sbpEvt_t *pMsg = ICall_malloc(sizeof(sbpEvt_t));

    // Create dynamic pointer to message.
    if (pMsg) {
        pMsg->hdr.event = SBP_NEW_DATA_EVT;
        pMsg->hdr.state = 0;
        pMsg->pData = data;

        // Enqueue the message.
        uint8_t ret = Util_enqueueMsg(dataMsgQueue, dataEvent, (uint8_t *) pMsg);
    }
}
