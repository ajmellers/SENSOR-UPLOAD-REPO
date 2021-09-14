/**********************************************************************************************
 * Filename:       motionService.h
 *
 * Description:    This file contains the motionService service definitions and
 *                 prototypes.
 *
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *************************************************************************************************/


#ifndef _MOTIONSERVICE_H_
#define _MOTIONSERVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
* CONSTANTS
*/
// Service UUID
#define MOTIONSERVICE_SERV_UUID 0xBABE

//  Characteristic defines

#define DATA_LEN_MULTIPLIER      1
#define MOTIONSERVICE_DATA      0
#define MOTIONSERVICE_DATA_UUID 0xBEEF
#define MOTIONSERVICE_DATA_LEN  DATA_LEN_MULTIPLIER*230

#define MOTIONSERVICE_MOTIONCONFIG      1
#define MOTIONSERVICE_MOTIONCONFIG_UUID 0xBEEE
#define MOTIONSERVICE_MOTIONCONFIG_LEN  19


#define MOTIONSERVICE_SENDINGFLAG      2
#define MOTIONSERVICE_SENDINGFLAG_UUID 0xBEE2

#define MOTIONSERVICE_SENDINGACK      3
#define MOTIONSERVICE_SENDINGACK_UUID 0xBEE3

#define MOTIONSERVICE_MEMORYSTATE      4
#define MOTIONSERVICE_MEMORYSTATE_UUID 0xBEE4

#define MOTIONSERVICE 0x00000001
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*motionServiceChange_t)( uint8 paramID );

typedef struct
{
  motionServiceChange_t        pfnChangeCb;  // Called when characteristic value changes
} motionServiceCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * MotionService_AddService- Initializes the MotionService service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t MotionService_AddService( uint32 services );

/*
 * MotionService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t MotionService_RegisterAppCBs( motionServiceCBs_t *appCallbacks );

/*
 * MotionService_SetParameter - Set a MotionService parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t MotionService_SetParameter( uint8 param, uint16 len, void *value );

/*
 * MotionService_GetParameter - Get a MotionService parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t MotionService_GetParameter( uint8 param, void *value );


static uint8_t sendingFlagZero = 0;
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _MOTIONSERVICE_H_ */
