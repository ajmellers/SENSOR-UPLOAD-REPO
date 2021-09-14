/**********************************************************************************************
 * Filename:       motionService.c
 *
 * Description:    This file contains the implementation of the service.
 *
 * Copyright (c) 2015-2018 MedEngine GmbH
 *
 *************************************************************************************************/
/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <ti/display/Display.h>
#include "bcomdef.h"
#include "osal.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "motionService.h"
#include "simple_peripheral.h"


/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * CONSTANTS
 */
/*********************************************************************
 * TYPEDEFS
 */
/*********************************************************************
 * GLOBAL VARIABLES
 */
// motionService Service UUID
CONST uint8_t motionServiceUUID[ATT_BT_UUID_SIZE] = {
        LO_UINT16(MOTIONSERVICE_SERV_UUID), HI_UINT16(MOTIONSERVICE_SERV_UUID) };
// motionData UUID
CONST uint8_t motionService_DataUUID[ATT_UUID_SIZE] = {
        TI_BASE_UUID_128(MOTIONSERVICE_DATA_UUID) };
// motionConfig UUID
CONST uint8_t motionService_ConfigUUID[ATT_UUID_SIZE] = {
        TI_BASE_UUID_128(MOTIONSERVICE_MOTIONCONFIG_UUID) };
// sendingFlag UUID
CONST uint8_t motionService_SendingFlagUUID[ATT_UUID_SIZE] = {
        TI_BASE_UUID_128(MOTIONSERVICE_SENDINGFLAG_UUID) };
// sendingAck UUID
CONST uint8_t motionService_SendingAckUUID[ATT_UUID_SIZE] = {
        TI_BASE_UUID_128(MOTIONSERVICE_SENDINGACK_UUID) };
// memoryState UUID
CONST uint8_t motionService_MemoryStateUUID[ATT_UUID_SIZE] = {
        TI_BASE_UUID_128(MOTIONSERVICE_MEMORYSTATE_UUID) };
extern Display_Handle dispHandle;


static uint8_t ackcounter = 0;
/*********************************************************************
 * LOCAL VARIABLES
 */
static motionServiceCBs_t *pAppCBs = NULL;
/*********************************************************************
 * Profile Attributes - variables
 */
// Service declaration
static CONST gattAttrType_t motionServiceDecl = {ATT_BT_UUID_SIZE, motionServiceUUID};
// Characteristic "MotionData" Properties (for declaration)
static uint8_t motionService_MotionDataProps = GATT_PROP_READ;
// Characteristic "MotionData" Value variable
static uint8_t motionService_MotionDataVal[MOTIONSERVICE_DATA_LEN] = {0};
// Characteristic "MotionData" User Description
static uint8 motionService_MotionDataDesc[] = "Motion Data";

// Characteristic "MotionConfig" Properties (for declaration)
static uint8_t motionService_MotionConfigProps = GATT_PROP_READ | GATT_PROP_WRITE;
// Characteristic "MotionConfig" Value variable
static uint8_t motionService_MotionConfigVal[MOTIONSERVICE_MOTIONCONFIG_LEN] = {0};
// Characteristic "MotionConfig" User Description
static uint8 motionService_MotionConfigDesc[] = "Motion Config";

// Characteristic "SendingFlag" Properties (for declaration)
static uint8_t motionService_SendingFlagProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic "SendingFlag" Value variable
static uint8 motionService_SendingFlagVal = 0;
// Characteristic "SendingFlag" CCCD
static gattCharCfg_t *motionService_SendingFlagConfig;
// Characteristic "SendingFlag" User Description
static uint8 motionService_SendingFlagDesc[] = "Sending Flag";

// Characteristic "SendingAck" Properties (for declaration)
static uint8_t motionService_SendingAckProps = GATT_PROP_READ | GATT_PROP_WRITE_NO_RSP;
// Characteristic "SendingAck" Value variable
static volatile uint8 motionService_SendingAckVal = 0;
// Characteristic "SendingAck" User Description
static uint8 motionService_SendingAckDesc[] = "Sending Ack";

// Characteristic "MemoryState" Properties (for declaration)
static uint8_t motionService_MemoryStateProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic "MemoryState" Value variable
static uint8 motionService_MemoryStateVal = 0;
// Characteristic "MemoryState" CCCD
static gattCharCfg_t *motionService_MemoryStateConfig;
// Characteristic "MemoryState" User Description
static uint8 motionService_MemoryStateDesc[] = "MemoryState";
/*********************************************************************
 * Profile Attributes - Table
 */
static gattAttribute_t motionServiceAttrTbl[] = {
        // motionService Service Declaration
        {
            {ATT_BT_UUID_SIZE, primaryServiceUUID},
            GATT_PERMIT_READ,
            0,
            (uint8_t *)&motionServiceDecl
        },
        // MotionData Characteristic Declaration
        {
            {ATT_BT_UUID_SIZE, characterUUID},
            GATT_PERMIT_READ,
            0,
            &motionService_MotionDataProps
        },
        // MotionData Characteristic Value
        {
            {ATT_UUID_SIZE, motionService_DataUUID},
            GATT_PERMIT_READ,
            0,
            motionService_MotionDataVal
        },
        // MotionData  Characteristic Value
        {
            { ATT_BT_UUID_SIZE, charUserDescUUID },
            GATT_PERMIT_READ,
            0,
            motionService_MotionDataDesc
        },
        // MotionConfig Characteristic Declaration
        {
            {ATT_BT_UUID_SIZE, characterUUID},
            GATT_PERMIT_READ,
            0,
            &motionService_MotionConfigProps
        },
        // MotionConfig Characteristic Value
        {
            {ATT_UUID_SIZE, motionService_ConfigUUID},
            GATT_PERMIT_READ | GATT_PERMIT_WRITE,
            0,
            motionService_MotionConfigVal
        },
        // MotionConfig  Characteristic Value
        {
            { ATT_BT_UUID_SIZE, charUserDescUUID },
            GATT_PERMIT_READ,
            0,
            motionService_MotionConfigDesc
        },
        // SendingFlag Characteristic Declaration
        {
            {ATT_BT_UUID_SIZE, characterUUID},
            GATT_PERMIT_READ,
            0,
            &motionService_SendingFlagProps
        },
        // SendingFlag Characteristic Value
        {
            {ATT_UUID_SIZE,    motionService_SendingFlagUUID},
            GATT_PERMIT_READ,
            0,
            &motionService_SendingFlagVal
        },
        // SendingFlag CCCD
        {
            {ATT_BT_UUID_SIZE, clientCharCfgUUID},
            GATT_PERMIT_READ | GATT_PERMIT_WRITE,
            0,
            (uint8 *)&motionService_SendingFlagConfig
        },
        // SendingFlag  Characteristic Value
        {
            { ATT_BT_UUID_SIZE, charUserDescUUID },
            GATT_PERMIT_READ,
            0,
            motionService_SendingFlagDesc
        },
        // SendingAck Characteristic Declaration
        {
            {ATT_BT_UUID_SIZE, characterUUID},
            GATT_PERMIT_READ,
            0,
            &motionService_SendingAckProps
        },
        // SendingAck  Characteristic Value
        {
            {ATT_UUID_SIZE,    motionService_SendingAckUUID},
            GATT_PERMIT_READ | GATT_PERMIT_WRITE,
            0,
            (uint8 *)&motionService_SendingAckVal
        },
        // SendingAck  Characteristic Description
        {
            { ATT_BT_UUID_SIZE, charUserDescUUID },
            GATT_PERMIT_READ,
            0,
            motionService_SendingAckDesc
        },
        // MemoryState Characteristic Declaration
        {
            {ATT_BT_UUID_SIZE, characterUUID},
            GATT_PERMIT_READ,
            0,
            &motionService_MemoryStateProps
        },
        // MemoryState  Characteristic Value
        {
            {ATT_UUID_SIZE,    motionService_MemoryStateUUID},
            GATT_PERMIT_READ,
            0,
            &motionService_MemoryStateVal
        },
        // MemoryState CCCD
        {
            {ATT_BT_UUID_SIZE, clientCharCfgUUID},
            GATT_PERMIT_READ | GATT_PERMIT_WRITE,
            0,
            (uint8 *)&motionService_MemoryStateConfig
        },
        // MemoryState  Characteristic Description
        {
            { ATT_BT_UUID_SIZE, charUserDescUUID },
            GATT_PERMIT_READ,
            0,
            motionService_MemoryStateDesc
        }

};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t motionService_ReadAttrCB(uint16 connHandle,
                                          gattAttribute_t *pAttr, uint8 *pValue,
                                          uint16 *pLen, uint16 offset,
                                          uint16 maxLen, uint8 method);

static bStatus_t motionService_WriteAttrCB(uint16 connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 len,
                                           uint16 offset, uint8 method);
/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t motionServiceCBs = {
        motionService_ReadAttrCB, // Read callback function pointer
        motionService_WriteAttrCB, // Write callback function pointer
        NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 *
 *
 * MotionService_AddService- Initializes the MotionService service by registering
 *          GATT attributes with the GATT server.
 *
 */
bStatus_t MotionService_AddService(uint32 services) {
    uint8_t status;
    // Allocate Client Characteristic Configuration table
    motionService_MemoryStateConfig = (gattCharCfg_t *) ICall_malloc(
            sizeof(gattCharCfg_t) * linkDBNumConns);
    if (motionService_MemoryStateConfig == NULL) {
        return (bleMemAllocError);
    }
    motionService_SendingFlagConfig = (gattCharCfg_t *) ICall_malloc(
            sizeof(gattCharCfg_t) * linkDBNumConns);
    if (motionService_SendingFlagConfig == NULL) {
        return (bleMemAllocError);
    }
    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg(INVALID_CONNHANDLE,
                            motionService_MemoryStateConfig);
    GATTServApp_InitCharCfg(INVALID_CONNHANDLE,
                            motionService_SendingFlagConfig);

    // Register GATT attribute list and CBs with GATT Server App
    if ( services & MOTIONSERVICE ) {
        status = GATTServApp_RegisterService(motionServiceAttrTbl,
                                             GATT_NUM_ATTRS(motionServiceAttrTbl),
                                             GATT_MAX_ENCRYPT_KEY_SIZE,
                                             &motionServiceCBs);
    } else {
        status = SUCCESS;
    }

    return (status);
}

/*********************************************************************
 * MotionService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t MotionService_RegisterAppCBs(motionServiceCBs_t *appCallbacks) {
    if (appCallbacks) {
        pAppCBs = appCallbacks;
        return (SUCCESS);
    } else {
        return (bleAlreadyInRequestedMode);
    }
}

/****
 * *****************************************************************
 * MotionService_SetParameter - Set a MotionService parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t MotionService_SetParameter(uint8 param, uint16 len, void *value) {
    bStatus_t ret = SUCCESS;

    switch (param) {
        case MOTIONSERVICE_DATA:
            if (len == MOTIONSERVICE_DATA_LEN) {
                memcpy(motionService_MotionDataVal, value, len);
            } else {
                ret = bleInvalidRange;
            }
            break;

        case MOTIONSERVICE_MOTIONCONFIG:
            if (len == MOTIONSERVICE_MOTIONCONFIG_LEN) {
                memcpy(motionService_MotionConfigVal, value, len);
            } else {
                ret = bleInvalidRange;
            }
            break;

        case MOTIONSERVICE_SENDINGFLAG:
            if ( len == sizeof ( uint8 ) ) {

                uint8_t temp = *((uint8_t*)value);
                if(motionService_SendingFlagVal != temp) {
                    Display_print1(dispHandle, 9, 0, "Entered sending flag change to %d deeper", temp);
                    motionService_SendingFlagVal = temp;

                    Display_print0(dispHandle, 9, 0, "Processing flag change");
                    GATTServApp_ProcessCharCfg(motionService_SendingFlagConfig,
                                               (uint8_t *)&motionService_SendingFlagVal,
                                               FALSE,
                                               motionServiceAttrTbl,
                                               GATT_NUM_ATTRS(motionServiceAttrTbl),
                                               INVALID_TASK_ID,
                                               motionService_ReadAttrCB);
                    Display_print0(dispHandle, 9, 0, "Processedflag change");
                }
            } else {
                ret = bleInvalidRange;
            }
            break;

        case MOTIONSERVICE_SENDINGACK:
            if ( len == sizeof ( uint8 ) ) {
                motionService_SendingAckVal = *((uint8*)value);
            } else {
                ret = bleInvalidRange;
            }
            break;

        case MOTIONSERVICE_MEMORYSTATE:
            if ( len == sizeof ( uint8 ) ) {
                uint8 temp = *((uint8*)value);
                if(motionService_MemoryStateVal != temp) {
                    motionService_MemoryStateVal = temp;

                    GATTServApp_ProcessCharCfg(motionService_MemoryStateConfig,
                                               (uint8_t *)&motionService_MemoryStateVal,
                                               FALSE,
                                               motionServiceAttrTbl,
                                               GATT_NUM_ATTRS(motionServiceAttrTbl),
                                               INVALID_TASK_ID,
                                               motionService_ReadAttrCB);
                }
            } else {
                ret = bleInvalidRange;
            }
            break;

        default:
            ret = INVALIDPARAMETER;
            break;
    }
    return ret;
}

/*********************************************************************
 * MotionService_GetParameter - Get a MotionService parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t MotionService_GetParameter(uint8 param, void *value) {
    bStatus_t ret = SUCCESS;
    switch (param) {
        case MOTIONSERVICE_DATA:
            memcpy(value, motionService_MotionDataVal,
                   MOTIONSERVICE_DATA_LEN);
            break;

        case MOTIONSERVICE_MOTIONCONFIG:
            memcpy(value, motionService_MotionConfigVal,
                   MOTIONSERVICE_MOTIONCONFIG_LEN);
            break;

        case MOTIONSERVICE_SENDINGFLAG:
            *((uint8*)value) = motionService_SendingFlagVal;
            break;

        case MOTIONSERVICE_SENDINGACK:
            *((uint8*)value) = motionService_SendingAckVal;
            break;

        case MOTIONSERVICE_MEMORYSTATE:
            *((uint8*)value) = motionService_MemoryStateVal;
            break;

        default:
            ret = INVALIDPARAMETER;
            break;
    }
    return ret;
}

/*********************************************************************
 * @fn          motionService_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t motionService_ReadAttrCB(uint16 connHandle,
                                          gattAttribute_t *pAttr, uint8 *pValue,
                                          uint16 *pLen, uint16 offset,
                                          uint16 maxLen, uint8 method) {
    bStatus_t status = SUCCESS;
    uint8_t paramID = 0xFF;
    if(*(pAttr->type.uuid)!= 0)
        Display_print1(dispHandle, 7, 30, "Read request: %d", *(pAttr->type.uuid));
    // See if request is regarding the MotionData Characteristic Value
    if (!memcmp(pAttr->type.uuid, motionService_DataUUID,
                pAttr->type.len)) {
        if (offset > MOTIONSERVICE_DATA_LEN) // Prevent malicious ATT ReadBlob offsets.
        {
            status = ATT_ERR_INVALID_OFFSET;
        } else {
            *pLen = MIN(maxLen, MOTIONSERVICE_DATA_LEN - offset); // Transmit as much as possible
            memcpy(pValue, pAttr->pValue + offset, *pLen);
            paramID = MOTIONSERVICE_DATA;
        }
    }
        // See if request is regarding the MotionConfig Characteristic Value
    else if (!memcmp(pAttr->type.uuid, motionService_ConfigUUID,
                     pAttr->type.len)) {
        if (offset > MOTIONSERVICE_MOTIONCONFIG_LEN) // Prevent malicious ATT ReadBlob offsets.
        {
            status = ATT_ERR_INVALID_OFFSET;
        } else {
            *pLen = MIN(maxLen, MOTIONSERVICE_MOTIONCONFIG_LEN - offset); // Transmit as much as possible
            memcpy(pValue, pAttr->pValue + offset, *pLen);
        }
    }
        // See if request is regarding the SendingFlag Characteristic Value
    else if (!memcmp(pAttr->type.uuid, motionService_SendingFlagUUID, pAttr->type.len)) {
        Display_print0(dispHandle, 9, 0, "Entered callback");
        if (offset > 0) { // Prevent malicious ATT ReadBlob offsets.
            status = ATT_ERR_INVALID_OFFSET;
        } else {
            *pLen = 1;
            *pValue = *pAttr->pValue;
        }
    }
        // See if request is regarding the SendingAck Characteristic Value
    else if (!memcmp(pAttr->type.uuid, motionService_SendingAckUUID, pAttr->type.len)) {
        if (offset > 0) { // Prevent malicious ATT ReadBlob offsets.
            status = ATT_ERR_INVALID_OFFSET;
        } else {
            *pLen = 1;
            pValue[0] = *pAttr->pValue;
        }
    }
        // See if request is regarding the MemoryState Characteristic Value
    else if (!memcmp(pAttr->type.uuid, motionService_MemoryStateUUID, pAttr->type.len)) {
        if (offset > 0) { // Prevent malicious ATT ReadBlob offsets.
            status = ATT_ERR_INVALID_OFFSET;
        } else {
            *pLen = 1;
            pValue[0] = *pAttr->pValue;
        }
    } else {
        // If we get here, that means you've forgotten to add an if clause for a
        // characteristic value attribute in the attribute table that has READ permissions.
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
    }
    if (paramID != 0xFF)
        if (pAppCBs && pAppCBs->pfnChangeCb)
            pAppCBs->pfnChangeCb(paramID);
    return status;
}

/*********************************************************************
 * @fn      motionService_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t motionService_WriteAttrCB(uint16 connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 len,
                                           uint16 offset, uint8 method) {
    bStatus_t status = SUCCESS;
    uint8_t paramID = 0xFF;
    // See if request is regarding a Client Characterisic Configuration
    if (!memcmp(pAttr->type.uuid, clientCharCfgUUID, pAttr->type.len)) {
        // Allow only notifications.
        status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                offset, GATT_CLIENT_CFG_NOTIFY);
    }
        // See if request is regarding the MotionConfig Characteristic Value
    else if (!memcmp(pAttr->type.uuid, motionService_ConfigUUID,
                     pAttr->type.len)) {
        if (offset + len > MOTIONSERVICE_MOTIONCONFIG_LEN) {
            status = ATT_ERR_INVALID_OFFSET;
        } else {
            // Copy pValue into the variable we point to from the attribute table.
            memcpy(pAttr->pValue + offset, pValue, len);
            // Only notify application if entire expected value is written
            if (offset + len == MOTIONSERVICE_MOTIONCONFIG_LEN)
                paramID = MOTIONSERVICE_MOTIONCONFIG;
        }
    }
        // See if request is regarding the SendingAck Characteristic Value
    else if (!memcmp(pAttr->type.uuid, motionService_SendingAckUUID, pAttr->type.len)) {
        if (offset + len > 1) {
            status = ATT_ERR_INVALID_OFFSET;
        } else {
            *((uint8_t *)(pAttr->pValue+offset)) = *((uint8_t *)pValue);

            Display_print1(dispHandle, 8, 0, "Ack write with value %d", *pValue);
            if((*pValue) == 1) {
                ackcounter++;
                Display_print1(dispHandle, 8, 0, "Ack write 1, counter: %d", ackcounter);
            }
            // Only notify application if entire expected value is written
            if (offset + len == 1) {
                paramID = MOTIONSERVICE_SENDINGACK;
            }
        }

    } else {
        // If we get here, that means you've forgotten to add an if clause for a
        // characteristic value attribute in the attribute table that has WRITE permissions.
        status = ATT_ERR_ATTR_NOT_FOUND;
    }

    // Let the application know something changed (if it did) by using the
    // callback it registered earlier (if it did).
    if (paramID != 0xFF)
        if (pAppCBs && pAppCBs->pfnChangeCb)
            pAppCBs->pfnChangeCb(paramID); // Call app function from stack task context.
    return status;
}
