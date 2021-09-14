#include <string.h>
#include <ti/display/Display.h>
#include "bcomdef.h"
#include "linkdb.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"
#include "timeService.h"

CONST uint8_t timeServiceUUID[ATT_BT_UUID_SIZE] = {
        LO_UINT16(CURRENT_TIME_SERV_UUID), HI_UINT16(CURRENT_TIME_SERV_UUID) };

CONST uint8_t timeService_SyncUUID[ATT_UUID_SIZE] = {
        TI_BASE_UUID_128(CURRENT_TIME_UUID) };


static timeServiceCBs_t *pAppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */
// Service declaration
static CONST gattAttrType_t timeServiceDecl = {ATT_BT_UUID_SIZE, timeServiceUUID};
// Characteristic "TimeSync" Properties (for declaration)
static uint8_t timeService_SyncProps = GATT_PROP_READ | GATT_PROP_WRITE;
// Characteristic "TimeSync" Value variable
static uint8_t timeService_SyncVal[TIMESERVICE_DATA_LEN] = {0};
// Characteristic "TimeSync" User Description
static uint8 timeService_SyncDesc[] = "TimeSync";


extern Display_Handle dispHandle;


/*********************************************************************
 * Profile Attributes - Table
 */
static gattAttribute_t timeServiceAttrTbl[] = {
        // timeService Declaration
        {
            {ATT_BT_UUID_SIZE, primaryServiceUUID},
            GATT_PERMIT_READ,
            0,
            (uint8_t *)&timeServiceDecl
        },
        // TimeSync Characteristic Declaration
        {
            {ATT_BT_UUID_SIZE, characterUUID},
            GATT_PERMIT_READ,
            0,
            &timeService_SyncProps
        },
        // TimeSync Characteristic Value
        {
            {ATT_UUID_SIZE, timeService_SyncUUID},
            GATT_PERMIT_READ | GATT_PERMIT_WRITE,
            0,
            timeService_SyncVal
        },
        // TimeData  Characteristic Value
        {
            { ATT_BT_UUID_SIZE, charUserDescUUID },
            GATT_PERMIT_READ,
            0,
            timeService_SyncDesc
        }};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t timeService_ReadAttrCB(uint16 connHandle,
                                          gattAttribute_t *pAttr, uint8 *pValue,
                                          uint16 *pLen, uint16 offset,
                                          uint16 maxLen, uint8 method);

static bStatus_t timeService_WriteAttrCB(uint16 connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 len,
                                           uint16 offset, uint8 method);
/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t timeServiceCBs = {
        timeService_ReadAttrCB, // Read callback function pointer
        timeService_WriteAttrCB, // Write callback function pointer
        NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 *
 *
 * TimeService_AddService- Initializes the TimeService by registering
 *          GATT attributes with the GATT server.
 *
 */
bStatus_t TimeService_AddService(void) {
    uint8_t status;
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService(timeServiceAttrTbl,
                                         GATT_NUM_ATTRS(timeServiceAttrTbl),
                                         GATT_MAX_ENCRYPT_KEY_SIZE,
                                         &timeServiceCBs);
    return (status);
}

/*********************************************************************
 * TimeService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t TimeService_RegisterAppCBs(timeServiceCBs_t *appCallbacks) {
    if (appCallbacks) {
        pAppCBs = appCallbacks;
        return (SUCCESS);
    } else {
        return (bleAlreadyInRequestedMode);
    }
}

/****
 * *****************************************************************
 * TimeService_SetParameter - Set a TimeService parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t TimeService_SetParameter(uint8 param, uint8 len, void *value) {
    bStatus_t ret = SUCCESS;
    switch (param) {
        case TIMESERVICE_SYNC_DATA:
            if (len == TIMESERVICE_DATA_LEN) {
                memcpy(timeService_SyncVal, value, len);
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
 * TimeService_GetParameter - Get a TimeService parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t TimeService_GetParameter(uint8 param, void *value) {
    bStatus_t ret = SUCCESS;
    switch (param) {
        case TIMESERVICE_SYNC_DATA:
            memcpy(value, timeService_SyncVal, TIMESERVICE_DATA_LEN);
            break;

        default:
            ret = INVALIDPARAMETER;
            break;
    }
    return ret;
}

/*********************************************************************
 * @fn          timeService_ReadAttrCB
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
static bStatus_t timeService_ReadAttrCB(uint16 connHandle,
                                          gattAttribute_t *pAttr, uint8 *pValue,
                                          uint16 *pLen, uint16 offset,
                                          uint16 maxLen, uint8 method) {
    bStatus_t status = SUCCESS;
    // See if request is regarding the TimeData Characteristic Value
    if (!memcmp(pAttr->type.uuid, timeService_SyncUUID,
                pAttr->type.len)) {
        if (offset > TIMESERVICE_DATA_LEN) // Prevent malicious ATT ReadBlob offsets.
        {
            status = ATT_ERR_INVALID_OFFSET;
        } else {
            *pLen = MIN(maxLen, TIMESERVICE_DATA_LEN - offset); // Transmit as much as possible
            memcpy(pValue, pAttr->pValue + offset, *pLen);
        }
    } else {
        // If we get here, that means you've forgotten to add an if clause for a
        // characteristic value attribute in the attribute table that has READ permissions.
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
    }
    return status;
}

/*********************************************************************
 * @fn      timeService_WriteAttrCB
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
static bStatus_t timeService_WriteAttrCB(uint16 connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 len,
                                           uint16 offset, uint8 method) {
    bStatus_t status = SUCCESS;
    uint8_t paramID = 0xFF;
    if (!memcmp(pAttr->type.uuid, timeService_SyncUUID,
                     pAttr->type.len)) {
        if (offset + len > TIMESERVICE_DATA_LEN) {
            status = ATT_ERR_INVALID_OFFSET;
        } else {
            // Copy pValue into the variable we point to from the attribute table.
            memcpy(pAttr->pValue + offset, pValue, len);
            // Only notify application if entire expected value is written
            if (offset + len == TIMESERVICE_DATA_LEN)
                paramID = TIMESERVICE_SYNC_DATA;
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
