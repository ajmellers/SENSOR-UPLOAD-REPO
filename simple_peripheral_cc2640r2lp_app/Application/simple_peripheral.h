/******************************************************************************

 @file  simple_peripheral.h

 @brief This file contains the Simple Peripheral sample application
        definitions and prototypes.

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

#ifndef SIMPLEPERIPHERAL_H
#define SIMPLEPERIPHERAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "util.h"

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
*  TYPEDEFS
*/
typedef struct {
    appEvtHdr_t hdr;  // event header.
    uint8_t *pData;  // event data
} sbpEvt_t;

/*********************************************************************
 * CONSTANTS
 */
#define SBP_NEW_DATA_EVT                      0x0012


// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) for automatic
// parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     24

// Maximum connection interval (units of 1.25ms, 800=1000ms) for automatic
// parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     200

// Slave latency to use for automatic parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY         10

// Supervision timeout value (units of 10ms, 1000=10s) for automatic parameter
// update request
#define DEFAULT_DESIRED_CONN_TIMEOUT          500

// After the connection is formed, the peripheral waits until the central
// device asks for its preferred connection parameters
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_WAIT_REMOTE_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define NOTIFICATION_EVENT_INTERVAL            1000
#define MEMORY_STATE_EVENT_INTERVAL            1000
#define NEW_DATA_EVENT_INTERVAL                0.1
#define SENDING_FLAG_EVENT_INTERVAL            0.1

// Application specific event ID for HCI Connection Event End Events
#define SBP_HCI_CONN_EVT_END_EVT              0x0001


// Type of Display to open
#if !defined(Display_DISABLE_ALL)
#if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD != 0)
#define SBP_DISPLAY_TYPE Display_Type_LCD
#elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART != 0)
#define SBP_DISPLAY_TYPE Display_Type_UART
#else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
#define SBP_DISPLAY_TYPE 0 // Option not supported
#endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
#else // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
#define SBP_DISPLAY_TYPE 0 // No Display
#endif // !Display_DISABLE_ALL

// Task configuration
#define SBP_TASK_PRIORITY                     3

#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   800
#endif

// Application events
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_PAIRING_STATE_EVT                 0x0004
#define SBP_CONN_EVT                          0x0010

// Internal Events for RTOS application
#define SBP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SBP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define NOTIFICATION_EVENT                    Event_Id_11
#define MEMORY_STATE_EVENT                    Event_Id_12
#define CURRENT_TIME_EVENT                    Event_Id_13
#define NEW_DATA_EVENT                        Event_Id_14
#define SENDING_FLAG_EVENT                    Event_Id_15

// Bitwise OR of all events to pend on
#define SBP_ALL_EVENTS                        (SBP_ICALL_EVT        | \
                                               SBP_QUEUE_EVT        | \
                                               NOTIFICATION_EVENT   | \
                                               MEMORY_STATE_EVENT   | \
                                               CURRENT_TIME_EVENT   | \
                                               NEW_DATA_EVENT       | \
                                               SENDING_FLAG_EVENT)

// Set the register cause to the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_SET(RegisterCause) (connectionEventRegisterCauseBitMap |= RegisterCause )
// Remove the register cause from the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_REMOVE(RegisterCause) (connectionEventRegisterCauseBitMap &= (~RegisterCause) )
// Gets whether the current App is registered to the receive connection events
#define CONNECTION_EVENT_IS_REGISTERED (connectionEventRegisterCauseBitMap > 0)
// Gets whether the RegisterCause was registered to recieve connection event
#define CONNECTION_EVENT_REGISTRATION_CAUSE(RegisterCause) (connectionEventRegisterCauseBitMap & RegisterCause )

/*********************************************************************
 * TYPEDEFS
 */

typedef enum {
    APP_MSG_SERVICE_WRITE = 0, /* A characteristic value has been written     */
    APP_MSG_SERVICE_CFG, /* A characteristic configuration has changed  */
    APP_MSG_UPDATE_CHARVAL, /* Request from ourselves to update a value    */
    APP_MSG_GAP_STATE_CHANGE, /* The GAP / connection state has changed      */
    APP_MSG_SEND_PASSCODE, /* A pass-code/PIN is requested during pairing */
    APP_MSG_PERIODIC_TIMER,
} app_msg_types_t;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the Simple Peripheral.
 */
extern void SimplePeripheral_createTask(void);
extern void setMemoryState();


static ICall_SyncHandle syncEvent;


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEPERIPHERAL_H */
