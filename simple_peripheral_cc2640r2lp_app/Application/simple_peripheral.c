/******************************************************************************

 @file  simple_peripheral.c

 @brief This file contains the Simple Peripheral sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: cc2640r2

 ******************************************************************************

 Copyright (c) 2013-2020, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************


 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <xdc/cfg/global.h>
#include <stdint.h>
#include <string.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/display/Display.h>
#include <icall.h>
#include "att_rsp.h"
#include "MESpi.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"
#include "board_key.h"
#include "devinfoservice.h"
#include "simple_gatt_profile.h"
#include "ll_common.h"
#include "peripheral.h"
#include "motionService.h"
#include "ledService.h"
#include "ledManager.h"
#include "timeService.h"
#include "timeManager.h"
#include "simple_peripheral.h"
#include "BatteryGauge/MAX17043.h"
#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

static uint8_t configData[MOTIONSERVICE_MOTIONCONFIG_LEN] = {0};

static uint8_t ackFlag = 0;

extern const Semaphore_Handle ackSemaphore;

volatile static uint8_t previousSendingFlagState = 0;


/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.

// Clock instances for internal periodic events.
static Clock_Struct notificationClock;
static Clock_Struct memoryStateClock;
static Clock_Struct dataClock;
static Clock_Struct sendingFlagClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

// Scan response data (max size = 31 bytes)
static uint8_t scanRspData[] =
        {
                // complete name
                0x07,   // length of this data
                GAP_ADTYPE_LOCAL_NAME_COMPLETE,
                'F',
                'l',
                'y',
                't',
                't',
                'a',

                // connection interval range
                0x05,   // length of this data
                GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
                LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
                HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
                LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
                HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

                // Tx power level
                0x02,   // length of this data
                GAP_ADTYPE_POWER_LEVEL,
                0       // 0dBm
        };

// Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertising)
static uint8_t advertData[] =
        {
                // Flags: this field sets the device to use general discoverable
                // mode (advertises indefinitely) instead of general
                // discoverable mode (advertise for 30 seconds at a time)
                0x02,   // length of this data
                GAP_ADTYPE_FLAGS,
                DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

                0x03,
                GAP_ADTYPE_APPEARANCE,
                LO_UINT16(GAP_APPEARE_GENERIC_WATCH),
                HI_UINT16(GAP_APPEARE_GENERIC_WATCH),
                // service UUID, to notify central devices what services are included
                // in this peripheral
                0x03,
                GAP_ADTYPE_16BIT_MORE,
                LO_UINT16(MOTIONSERVICE_SERV_UUID),
                HI_UINT16(MOTIONSERVICE_SERV_UUID)
        };

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Flytta";


/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimplePeripheral_init(void);

static void SimplePeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg);

static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg);

static void SimplePeripheral_processAppMsg(sbpEvt_t *pMsg);

static void SimplePeripheral_processStateChangeEvt(gaprole_States_t newState);

static void syncEventClockHandler(UArg arg);

static void SimplePeripheral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status);

static void SimplePeripheral_processPairState(uint8_t state, uint8_t status);

static void SimplePeripheral_stateChangeCB(gaprole_States_t newState);

static uint8_t SimplePeripheral_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData);

static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport);

static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport);

static void SimplePeripheral_newDataEvent(uint8_t * data);

static void SimplePeripheral_processSendingDataMsg(sbpEvt_t *pMsg);

static void processAck(uint8_t paramID);

static void timeService_ValueChangeCB();

static void syncCurrentTime();

static void notificationTask();

static void zeroSendingFlag();


//static void setMemoryState();

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Peripheral GAPRole Callbacks
static gapRolesCBs_t SimplePeripheral_gapRoleCBs =
        {
                SimplePeripheral_stateChangeCB     // GAPRole State Change Callbacks
        };

// GAP Bond Manager Callbacks
// These are set to NULL since they are not needed. The application
// is set up to only perform justworks pairing.
static gapBondCBs_t simplePeripheral_BondMgrCBs =
        {
                NULL,
                SimplePeripheral_pairStateCB  // Pairing / Bonding state Callback
        };

// Motion Service callback handler.
// The type Motion_ServiceCBs_t is defined in motionService.h
static motionServiceCBs_t motion_ServiceCBs =
        {
             processAck
        };

// Time Service callback handler.
// The type Time_ServiceCBs_t is defined in timeService.h
static timeServiceCBs_t time_ServiceCBs =
        {
                 timeService_ValueChangeCB
        };

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * The following typedef and global handle the registration to connection event
 */
typedef enum {
    NOT_REGISTER = 0,
    FOR_AOA_SCAN = 1,
    FOR_ATT_RSP = 2,
    FOR_AOA_SEND = 4,
    FOR_TOF_SEND = 8
} connectionEventRegisterCause_u;

// Handle the registration and un-registration for the connection event, since only one can be registered.
uint32_t connectionEventRegisterCauseBitMap = NOT_REGISTER; //see connectionEventRegisterCause_u

/*********************************************************************
 * @fn      SimplePeripheral_RegistertToAllConnectionEvent()
 *
 * @brief   register to receive connection events for all the connection
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t SimplePeripheral_RegistertToAllConnectionEvent(connectionEventRegisterCause_u connectionEventRegisterCause) {
    bStatus_t status = SUCCESS;

    // in case  there is no registration for the connection event, make the registration
    if (!CONNECTION_EVENT_IS_REGISTERED) {
        status = GAP_RegisterConnEventCb(SimplePeripheral_connEvtCB, GAP_CB_REGISTER, LINKDB_CONNHANDLE_ALL);
    }
    if (status == SUCCESS) {
        //add the reason bit to the bitamap.
        CONNECTION_EVENT_REGISTER_BIT_SET(connectionEventRegisterCause);
    }

    return (status);
}

/*********************************************************************
 * @fn      SimplePeripheral_UnRegistertToAllConnectionEvent()
 *
 * @brief   Unregister connection events
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t
SimplePeripheral_UnRegistertToAllConnectionEvent(connectionEventRegisterCause_u connectionEventRegisterCause) {
    bStatus_t status = SUCCESS;

    CONNECTION_EVENT_REGISTER_BIT_REMOVE(connectionEventRegisterCause);
    // in case  there is no more registration for the connection event than unregister
    if (!CONNECTION_EVENT_IS_REGISTERED) {
        GAP_RegisterConnEventCb(SimplePeripheral_connEvtCB, GAP_CB_UNREGISTER, LINKDB_CONNHANDLE_ALL);
    }

    return (status);
}

/*********************************************************************
* @fn      SimplePeripheral_createTask
*
* @brief   Task creation function for the Simple Peripheral.
*
* @param   None.
*
* @return  None.
*/
void SimplePeripheral_createTask(void) {


    Task_Params taskParams;
    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = sbpTaskStack;
    taskParams.stackSize = SBP_TASK_STACK_SIZE;
    taskParams.priority = SBP_TASK_PRIORITY;

    Task_construct(&sbpTask, SimplePeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimplePeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimplePeripheral_init(void) {
    // ******************************************************************
    // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
    // ******************************************************************
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
    RCOSC_enableCalibration();
#endif // USE_RCOSC

#if defined( USE_FPGA )
    // configure RF Core SMI Data Link
    IOCPortConfigureSet(IOID_12, IOC_PORT_RFC_GPO0, IOC_STD_OUTPUT);
    IOCPortConfigureSet(IOID_11, IOC_PORT_RFC_GPI0, IOC_STD_INPUT);

    // configure RF Core SMI Command Link
    IOCPortConfigureSet(IOID_10, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_OUT, IOC_STD_OUTPUT);
    IOCPortConfigureSet(IOID_9, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_IN, IOC_STD_INPUT);

    // configure RF Core tracer IO
    IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT);
#else // !USE_FPGA
#if defined( DEBUG_SW_TRACE )
    // configure RF Core tracer IO
    IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT | IOC_CURRENT_4MA | IOC_SLEW_ENABLE);
#endif // DEBUG_SW_TRACE
#endif // USE_FPGA

    // Create an RTOS queue for message from profile to be sent to app.
    appMsgQueue = Util_constructQueue(&appMsg);

    // Create one-shot clocks for internal periodic events.
    Util_constructClock(&notificationClock, syncEventClockHandler,
                        NOTIFICATION_EVENT_INTERVAL, 0, false, NOTIFICATION_EVENT);

    Util_constructClock(&memoryStateClock, syncEventClockHandler,
                        MEMORY_STATE_EVENT_INTERVAL, 0, false, MEMORY_STATE_EVENT);

    Util_constructClock(&dataClock, syncEventClockHandler,
                        NEW_DATA_EVENT_INTERVAL, 0, false, NEW_DATA_EVENT);


    Util_constructClock(&sendingFlagClock, syncEventClockHandler,
                        SENDING_FLAG_EVENT_INTERVAL, 0, false, SENDING_FLAG_EVENT);

    dispHandle = Display_open(SBP_DISPLAY_TYPE, NULL);

    // Set GAP Parameters: After a connection was established, delay in seconds
    // before sending when GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE,...)
    // uses GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS or
    // GAPROLE_LINK_PARAM_UPDATE_INITIATE_APP_PARAMS
    // For current defaults, this has no effect.
    GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

    // Setup the Peripheral GAPRole Profile. For more information see the User's
    // Guide:
    // http://software-dl.ti.com/lprf/sdg-latest/html/
    {
        // By setting this to zero, the device will go into the waiting state after
        // being discoverable for 30.72 second, and will not being advertising again
        // until re-enabled by the application
        uint16_t advertOffTime = 0;

        uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
        uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
        uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

        GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                             &advertOffTime);

        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                             scanRspData);
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

        GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                             &enableUpdateRequest);
        GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                             &desiredMinInterval);
        GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                             &desiredMaxInterval);
        GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                             &desiredSlaveLatency);
        GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                             &desiredConnTimeout);
    }

    // Set the Device Name characteristic in the GAP GATT Service
    // For more information, see the section in the User's Guide:
    // http://software-dl.ti.com/lprf/sdg-latest/html
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Set GAP Parameters to set the advertising interval
    // For more information, see the GAP section of the User's Guide:
    // http://software-dl.ti.com/lprf/sdg-latest/html
    {
        // Use the same interval for general and limited advertising.
        // Note that only general advertising will occur based on the above configuration
        uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
    }

    // Setup the GAP Bond Manager. For more information see the section in the
    // User's Guide:
    // http://software-dl.ti.com/lprf/sdg-latest/html/
    {
        // Don't send a pairing request after connecting; the peer device must
        // initiate pairing
        uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        // Use authenticated pairing: require passcode.
        uint8_t mitm = FALSE;
        // This device only has display capabilities. Therefore, it will display the
        // passcode during pairing. However, since the default passcode is being
        // used, there is no need to display anything.
        uint8_t ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
        // Request bonding (storing long-term keys for re-encryption upon subsequent
        // connections without repairing)
        uint8_t bonding = TRUE;
        // Whether to replace the least recently used entry when bond list is full,
        // and a new device is bonded.
        // Alternative is pairing succeeds but bonding fails, unless application has
        // manually erased at least one bond.
        uint8_t replaceBonds = FALSE;
        uint8_t gapbondSecure = GAPBOND_SECURE_CONNECTION_NONE;

        GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
        GAPBondMgr_SetParameter(GAPBOND_LRU_BOND_REPLACEMENT, sizeof(uint8_t), &replaceBonds);
        GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &gapbondSecure);
    }

    // Initialize GATT attributes
    GGS_AddService(GATT_ALL_SERVICES);           // GAP GATT Service
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
    DevInfo_AddService();                        // Device Information Service
    MotionService_AddService(GATT_ALL_SERVICES);
    LEDService_AddService(GATT_ALL_SERVICES);
    TimeService_AddService();

    // Register callbacks with the generated services that
    // can generate events (writes received) to the application
    MotionService_RegisterAppCBs( &motion_ServiceCBs );
    TimeService_RegisterAppCBs( &time_ServiceCBs );

    // Placeholder variable for characteristic initialisation
    uint8_t intMotionData[MOTIONSERVICE_DATA_LEN] = {0};
    uint8_t intMotionConfig[MOTIONSERVICE_MOTIONCONFIG_LEN] = {0};
    uint8_t intTimeSyncData[TIMESERVICE_DATA_LEN] = {0};
    uint8_t intValue = 0;
    uint8_t intAckValue = 0;

    // Initialisation of characteristics in motionService that are readable.
    TimeService_SetParameter(TIMESERVICE_SYNC_DATA, TIMESERVICE_DATA_LEN, &intTimeSyncData);

    MotionService_SetParameter(MOTIONSERVICE_DATA, MOTIONSERVICE_DATA_LEN, &intMotionData);
    MotionService_SetParameter(MOTIONSERVICE_MOTIONCONFIG, MOTIONSERVICE_MOTIONCONFIG_LEN, &intMotionConfig);
    MotionService_SetParameter(MOTIONSERVICE_MEMORYSTATE, 1, &intValue);
    MotionService_SetParameter(MOTIONSERVICE_SENDINGFLAG, 1, &intValue);
    MotionService_SetParameter(MOTIONSERVICE_SENDINGACK, 1, &intAckValue);

    LEDService_SetParameter(LEDSERVICE_RED, 1, &intValue);
    LEDService_SetParameter(LEDSERVICE_WHITE, 1, &intValue);

    // Start the Device:
    // Please Notice that in case of wanting to use the GAPRole_SetParameter
    // function with GAPROLE_IRK or GAPROLE_SRK parameter - Perform
    // these function calls before the GAPRole_StartDevice use.
    // (because Both cases are updating the gapRole_IRK & gapRole_SRK variables).
    VOID GAPRole_StartDevice(&SimplePeripheral_gapRoleCBs);

    // Start Bond Manager and register callback
    VOID GAPBondMgr_Register(&simplePeripheral_BondMgrCBs);

    // Register with GAP for HCI/Host messages. This is needed to receive HCI
    // events. For more information, see the section in the User's Guide:
    // http://software-dl.ti.com/lprf/sdg-latest/html
    GAP_RegisterForMsgs(selfEntity);

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    //Set default values for Data Length Extension
    {
        //Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
#define APP_SUGGESTED_PDU_SIZE 119 //default is 27 octets(TX)
#define APP_SUGGESTED_TX_TIME 6*328 //default is 328us(TX)

        //This API is documented in hci.h
        //See the LE Data Length Extension section in the BLE-Stack User's Guide for information on using this command:
        //http://software-dl.ti.com/lprf/sdg-latest/html/cc2640/index.html
        HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
    }

#if !defined (USE_LL_CONN_PARAM_UPDATE)
    // Get the currently set local supported LE features
    // The HCI will generate an HCI event that will get received in the main
    // loop
    HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

    Display_print0(dispHandle, 0, 0, "BLE Peripheral");
}

/*********************************************************************
 * @fn      SimplePeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SimplePeripheral_taskFxn(UArg a0, UArg a1) {
    // Initialize application
    SimplePeripheral_init();

    // Application main loop
    for (;;) {
        uint32_t events;

        // Waits for an event to be posted associated with the calling thread.
        // Note that an event associated with a thread is posted when a
        // message is queued to the message receive queue of the thread
        events = Event_pend(syncEvent, Event_Id_NONE, SBP_ALL_EVENTS,
                            ICALL_TIMEOUT_FOREVER);

        if (events) {
            ICall_EntityID dest;
            ICall_ServiceEnum src;
            ICall_HciExtEvt *pMsg = NULL;

            // Fetch any available messages that might have been sent from the stack
            if (ICall_fetchServiceMsg(&src, &dest,
                                      (void **) &pMsg) == ICALL_ERRNO_SUCCESS) {
                uint8 safeToDealloc = TRUE;

                if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity)) {
                    ICall_Stack_Event *pEvt = (ICall_Stack_Event *) pMsg;

                    if (pEvt->signature != 0xffff) {
                        // Process inter-task message
                        safeToDealloc = SimplePeripheral_processStackMsg((ICall_Hdr *) pMsg);
                    }
                }

                if (pMsg && safeToDealloc) {
                    ICall_freeMsg(pMsg);
                }
            }

            // If RTOS queue is not empty, process app message.
            if (events & SBP_QUEUE_EVT) {
                while (!Queue_empty(appMsgQueue)) {
                    sbpEvt_t *pMsg = (sbpEvt_t *) Util_dequeueMsg(appMsgQueue);
                    if (pMsg) {
                        // Process message.
                        SimplePeripheral_processAppMsg(pMsg);

                        // Free the space from the message.
                        ICall_free(pMsg);
                    }
                }
            }

            if (events & MEMORY_STATE_EVENT) {
                Util_startClock(&memoryStateClock);
                Display_print0(dispHandle, 0, 30, "Setting memory state");
                setMemoryState();
            }

            if (events & NOTIFICATION_EVENT) {
                Util_startClock(&notificationClock);
                Display_print0(dispHandle, 0, 30, "Notifying");
                notificationTask();
            }

            if (events & CURRENT_TIME_EVENT) {
                syncCurrentTime();
            }

            if (events & SENDING_FLAG_EVENT) {
                Util_stopClock(&sendingFlagClock);
                zeroSendingFlag();
            }

            if (events & NEW_DATA_EVENT) {
                Util_startClock(&dataClock);
                sbpEvt_t *pMsg = (sbpEvt_t *) Util_dequeueMsg(dataMsgQueue);

                if (pMsg) {
                    // Process message.
                    SimplePeripheral_processSendingDataMsg(pMsg);

                    // Free the space from the message.
                    ICall_free(pMsg);
                }
            }
        }
    }
}

/*********************************************************************
 * @fn      SimplePeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg) {
    uint8_t safeToDealloc = TRUE;

    switch (pMsg->event) {
        case GATT_MSG_EVENT:
            // Process GATT message
            safeToDealloc = SimplePeripheral_processGATTMsg((gattMsgEvent_t *) pMsg);
            break;

        case HCI_GAP_EVENT_EVENT: {

            // Process HCI message
            switch (pMsg->status) {
                case HCI_COMMAND_COMPLETE_EVENT_CODE:
                    // Process HCI Command Complete Event
                {

#if !defined (USE_LL_CONN_PARAM_UPDATE)
                    // This code will disable the use of the LL_CONNECTION_PARAM_REQ
                    // control procedure (for connection parameter updates, the
                    // L2CAP Connection Parameter Update procedure will be used
                    // instead). To re-enable the LL_CONNECTION_PARAM_REQ control
                    // procedures, define the symbol USE_LL_CONN_PARAM_UPDATE
                    // The L2CAP Connection Parameter Update procedure is used to
                    // support a delta between the minimum and maximum connection
                    // intervals required by some iOS devices.

                    // Parse Command Complete Event for opcode and status
                    hciEvt_CmdComplete_t *command_complete = (hciEvt_CmdComplete_t *) pMsg;
                    uint8_t pktStatus = command_complete->pReturnParam[0];

                    //find which command this command complete is for
                    switch (command_complete->cmdOpcode) {
                        case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES: {
                            if (pktStatus == SUCCESS) {
                                uint8_t featSet[8];

                                // Get current feature set from received event (bits 1-9
                                // of the returned data
                                memcpy(featSet, &command_complete->pReturnParam[1], 8);

                                // Clear bit 1 of byte 0 of feature set to disable LL
                                // Connection Parameter Updates
                                CLR_FEATURE_FLAG(featSet[0], LL_FEATURE_CONN_PARAMS_REQ);

                                // Update controller with modified features
                                HCI_EXT_SetLocalSupportedFeaturesCmd(featSet);
                            }
                        }
                            break;

                        default:
                            //do nothing
                            break;
                    }
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

                }
                    break;

                case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
                    AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR, 0);
                    break;

                default:
                    break;
            }
        }
            break;

        default:
            // do nothing
            break;

    }

    return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimplePeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg) {
    // See if GATT server was unable to transmit an ATT response
    if (attRsp_isAttRsp(pMsg)) {
        // No HCI buffer was available. Let's try to retransmit the response
        // on the next connection event.
        if (SimplePeripheral_RegistertToAllConnectionEvent(FOR_ATT_RSP) == SUCCESS) {
            // Don't free the response message yet
            return (FALSE);
        }
    } else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT) {
        // ATT request-response or indication-confirmation flow control is
        // violated. All subsequent ATT requests or indications will be dropped.
        // The app is informed in case it wants to drop the connection.

        // Display the opcode of the message that caused the violation.
        Display_print1(dispHandle, 5, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    } else if (pMsg->method == ATT_MTU_UPDATED_EVENT) {
        // MTU size updated
        Display_print1(dispHandle, 5, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }

    // Free message payload. Needed only for ATT Protocol messages
    GATT_bm_free(&pMsg->msg, pMsg->method);

    // It's safe to free the incoming message
    return (TRUE);
}

/*********************************************************************
 * @fn      SimplePeripheral_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport) {

    if (CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_ATT_RSP)) {
        // The GATT server might have returned a blePending as it was trying
        // to process an ATT Response. Now that we finished with this
        // connection event, let's try sending any remaining ATT Responses
        // on the next connection event.
        // Try to retransmit pending ATT Response (if any)
        if (attRsp_sendAttRsp() == SUCCESS) {
            // Disable connection event end notice
            SimplePeripheral_UnRegistertToAllConnectionEvent(FOR_ATT_RSP);
        }
    }

}

/*********************************************************************
 * @fn      SimplePeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimplePeripheral_processAppMsg(sbpEvt_t *pMsg) {
    switch (pMsg->hdr.event) {
        case SBP_STATE_CHANGE_EVT: {
            SimplePeripheral_processStateChangeEvt((gaprole_States_t) pMsg->
                    hdr.state);
        }
            break;

            // Pairing event
        case SBP_PAIRING_STATE_EVT: {
            SimplePeripheral_processPairState(pMsg->hdr.state, *pMsg->pData);

            ICall_free(pMsg->pData);
            break;
        }

        case SBP_CONN_EVT: {
            SimplePeripheral_processConnEvt((Gap_ConnEventRpt_t * )(pMsg->pData));

            ICall_free(pMsg->pData);
            break;
        }

        default:
            // Do nothing.
            break;
    }
}

/*********************************************************************
 * @fn      SimplePeripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimplePeripheral_stateChangeCB(gaprole_States_t newState) {
    SimplePeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState, NULL);
}

/*********************************************************************
 * @fn      SimplePeripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimplePeripheral_processStateChangeEvt(gaprole_States_t newState) {
#ifdef PLUS_BROADCASTER
    static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

    switch (newState) {
        case GAPROLE_STARTED: {
            uint8_t ownAddress[B_ADDR_LEN];
            uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

            GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

            // use 6 bytes of device address for 8 bytes of system ID value
            systemId[0] = ownAddress[0];
            systemId[1] = ownAddress[1];
            systemId[2] = ownAddress[2];

            // set middle bytes to zero
            systemId[4] = 0x00;
            systemId[3] = 0x00;

            // shift three bytes up
            systemId[7] = ownAddress[5];
            systemId[6] = ownAddress[4];
            systemId[5] = ownAddress[3];

            DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

            // Display device address
            Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(ownAddress));
            Display_print0(dispHandle, 2, 0, "Initialized");

            // Device starts advertising upon initialization of GAP
            uint8_t initialAdvertEnable = TRUE;
            // Set the Peripheral GAPRole Parameters
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &initialAdvertEnable);
        }
            break;

        case GAPROLE_ADVERTISING:
            Display_print0(dispHandle, 2, 0, "Advertising");
            break;

#ifdef PLUS_BROADCASTER
            // After a connection is dropped, a device in PLUS_BROADCASTER will continue
            // sending non-connectable advertisements and shall send this change of
            // state to the application.  These are then disabled here so that sending
            // connectable advertisements can resume.
            case GAPROLE_ADVERTISING_NONCONN:
              {
                uint8_t advertEnabled = FALSE;

                // Disable non-connectable advertising.
                GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                   &advertEnabled);

                advertEnabled = TRUE;

                // Enabled connectable advertising.
                GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                     &advertEnabled);

                // Reset flag for next connection.
                firstConnFlag = false;

                attRsp_freeAttRsp(bleNotConnected);
              }
              break;
#endif //PLUS_BROADCASTER

        case GAPROLE_CONNECTED: {
            linkDBInfo_t linkInfo;
            uint8_t numActive = 0;

            Util_startClock(&notificationClock);
            Util_startClock(&memoryStateClock);
            Util_startClock(&dataClock);

            numActive = linkDB_NumActive();

            // Use numActive to determine the connection handle of the last
            // connection
            if (linkDB_GetInfo(numActive - 1, &linkInfo) == SUCCESS) {
                Display_print1(dispHandle, 2, 0, "Num Conns: %d", (uint16_t) numActive);
                Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(linkInfo.addr));
            } else {
                uint8_t peerAddress[B_ADDR_LEN];

                GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

                Display_print0(dispHandle, 2, 0, "Connected");
                Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddress));
            }

#ifdef PLUS_BROADCASTER
            // Only turn advertising on for this state when we first connect
            // otherwise, when we go from connected_advertising back to this state
            // we will be turning advertising back on.
            if (firstConnFlag == false)
            {
              uint8_t advertEnabled = FALSE; // Turn on Advertising

              // Disable connectable advertising.
              GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                   &advertEnabled);

              // Set to true for non-connectable advertising.
              advertEnabled = TRUE;

              // Enable non-connectable advertising.
              GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                   &advertEnabled);
              firstConnFlag = true;
            }
#endif // PLUS_BROADCASTER
        }
            break;

        case GAPROLE_CONNECTED_ADV:
            Display_print0(dispHandle, 2, 0, "Connected Advertising");
            break;

        case GAPROLE_WAITING: {
            uint8_t advertReEnable = TRUE;

            Util_stopClock(&notificationClock);
            Util_stopClock(&memoryStateClock);
            Util_stopClock(&dataClock);

            attRsp_freeAttRsp(bleNotConnected);

            // Clear remaining lines
            Display_clearLines(dispHandle, 3, 5);

            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertReEnable);
            Display_print0(dispHandle, 2, 0, "Advertising");
        }
            break;

        case GAPROLE_WAITING_AFTER_TIMEOUT:
            attRsp_freeAttRsp(bleNotConnected);

            Display_print0(dispHandle, 2, 0, "Timed Out");

            // Clear remaining lines
            Display_clearLines(dispHandle, 3, 5);

#ifdef PLUS_BROADCASTER
            // Reset flag for next connection.
            firstConnFlag = false;
#endif // PLUS_BROADCASTER
            break;

        case GAPROLE_ERROR:
            Display_print0(dispHandle, 2, 0, "Error");
            break;

        default:
            Display_clearLine(dispHandle, 2);
            break;
    }

}

/*********************************************************************
 * @fn      SimplePeripheral_newDataEvent
 *
 * @brief   Process a new data to send.
 *
 * @param   data - data to send.
 *
 * @return  none
 */
static void SimplePeripheral_newDataEvent(uint8_t * data) {
//    if(!((data[18]==0 && data[19]==0 && data[20]==0 && data[21]==0 && data[22]==0) || (data[18]==0xFF && data[19]==0xFF && data[20]==0xFF && data[21]==0xFF && data[22]==0xFF )))
    MotionService_SetParameter(MOTIONSERVICE_DATA, MOTIONSERVICE_DATA_LEN, data);
    static uint8_t sendingFlag = 1;
    MotionService_SetParameter(MOTIONSERVICE_SENDINGFLAG, 1, &sendingFlag);
}

/*********************************************************************
 * @fn      SimplePeripheral_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimplePeripheral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status) {
    uint8_t *pData;

    // Allocate space for the event data.
    if ((pData = ICall_malloc(sizeof(uint8_t)))) {
        *pData = status;

        // Queue the event.
        SimplePeripheral_enqueueMsg(SBP_PAIRING_STATE_EVT, state, pData);
    }
}

/*********************************************************************
 * @fn      SimplePeripheral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimplePeripheral_processPairState(uint8_t state, uint8_t status) {
    if (state == GAPBOND_PAIRING_STATE_STARTED) {
        Display_print0(dispHandle, 2, 0, "Pairing started");
    } else if (state == GAPBOND_PAIRING_STATE_COMPLETE) {
        if (status == SUCCESS) {
            Display_print0(dispHandle, 2, 0, "Pairing success");
        } else {
            Display_print1(dispHandle, 2, 0, "Pairing fail: %d", status);
        }
    } else if (state == GAPBOND_PAIRING_STATE_BONDED) {
        if (status == SUCCESS) {
            Display_print0(dispHandle, 2, 0, "Bonding success");
        }
    } else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED) {
        if (status == SUCCESS) {
            Display_print0(dispHandle, 2, 0, "Bond save success");
        } else {
            Display_print1(dispHandle, 2, 0, "Bond save failed: %d", status);
        }
    }
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
static void syncEventClockHandler(UArg arg) {
    // Wake up the application.
    Event_post(syncEvent, arg);
}


/*********************************************************************
 * @fn      SimplePeripheral_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport) {
    // Enqueue the event for processing in the app context.
    if (SimplePeripheral_enqueueMsg(SBP_CONN_EVT, 0, (uint8_t *) pReport) == FALSE) {
        ICall_free(pReport);
    }

}

/*********************************************************************
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimplePeripheral_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData) {
    sbpEvt_t *pMsg = ICall_malloc(sizeof(sbpEvt_t));

    // Create dynamic pointer to message.
    if (pMsg) {
        pMsg->hdr.event = event;
        pMsg->hdr.state = state;
        pMsg->pData = pData;

        // Enqueue the message.
        return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *) pMsg);
    }

    return FALSE;
}

/*********************************************************************/

static void SimplePeripheral_processSendingDataMsg(sbpEvt_t *pMsg) {
    switch (pMsg->hdr.event) {
        case SBP_NEW_DATA_EVT: {
            SimplePeripheral_newDataEvent(pMsg->pData);

//            ICall_free(pMsg->pData);
            break;
        }

        default:
            // Do nothing.
            break;
    }
}


/*********************************************************************
 * @brief   set values to notify about battery and led
 *
 * @return  none
 */
static void notificationTask() {
    //read battery level
    uint8_t memoryState;
    memoryState = getStorageStatus();
    Display_print1(dispHandle, 4, 0, "Memory status: %d", memoryState);
    MAX17043StateOfCharge(&configData[8]);
    MotionService_SetParameter(MOTIONSERVICE_MEMORYSTATE, 1, &memoryState);

    MotionService_SetParameter(MOTIONSERVICE_MOTIONCONFIG, MOTIONSERVICE_MOTIONCONFIG_LEN, &configData);

    LEDService_GetParameter(LEDSERVICE_RED, &ledsState.redLed);
    LEDService_GetParameter(LEDSERVICE_WHITE, &ledsState.whiteLed);
    setRedLedPin(ledsState.redLed);
}

/*********************************************************************
 * @brief   Set memory state
 *
 * @return  none
 */
void setMemoryState() {
    static uint8_t memoryState;
    memoryState = getStorageStatus();
    MotionService_SetParameter(MOTIONSERVICE_MEMORYSTATE, 1, &memoryState);
}


/*********************************************************************
 * @brief   Callback handler for characteristic value changes in time services.
 *
 * @return  none
 */
static void timeService_ValueChangeCB() {
    Event_post(syncEvent, CURRENT_TIME_EVENT);
}

/*********************************************************************
 * @brief   synchronize current time
 *
 * @param   TimeController
 *
 * @return  none
 */
static void syncCurrentTime() {
    union CurrentTime *timeSync = ICall_malloc(sizeof(union CurrentTime));

    TimeService_GetParameter(TIMESERVICE_SYNC_DATA, timeSync->byteArrayForBLE);
    if (timeSync->exactTime.adj){
        timeSync->exactTime.adj = 0;
        updateCurrentTime(&timeSync->exactTime.timestamp);
    } else {
        getCurrentTime(timeSync);
    }
    TimeService_SetParameter(TIMESERVICE_SYNC_DATA, sizeof(union CurrentTime), timeSync->byteArrayForBLE);

    ICall_free(timeSync);
}

/*********************************************************************
 * @brief   process ack
 *
 * @param   paramID
 *
 * @return  none
 */
static void processAck(uint8_t paramID) {
    if (paramID == MOTIONSERVICE_SENDINGACK) {
        MotionService_GetParameter(MOTIONSERVICE_SENDINGACK, &ackFlag);
        Display_print1(dispHandle, 9, 0, "Posting semaphore: %d", ackFlag);
        if (ackFlag) {
            ackFlag=0;
            MotionService_SetParameter(MOTIONSERVICE_SENDINGACK, 1, &ackFlag);
            Event_post(syncEvent, SENDING_FLAG_EVENT);
            Semaphore_post(ackSemaphore);
            Display_print1(dispHandle, 9, 0, "Posted semaphore: %d", ackFlag);
        }
    }
    if (paramID == MOTIONSERVICE_DATA) {
        Display_print0(dispHandle, 10, 0, "MOTIONSERVICE DATA!!!!!!!!!!!!");
    }
}

/*********************************************************************
 * @brief   Zero sending flag
 *
 * @return  none
 */
static void zeroSendingFlag() {
    static uint8_t sendingFlag = 0;
    MotionService_SetParameter(MOTIONSERVICE_SENDINGFLAG, 1, &sendingFlag);
}


/*********************************************************************
*********************************************************************/
