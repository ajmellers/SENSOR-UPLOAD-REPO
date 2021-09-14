/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include "bcomdef.h"
#include "osal.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "ledService.h"

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
// ledService Service UUID
CONST uint8_t ledServiceUUID[ATT_BT_UUID_SIZE] = {
        LO_UINT16(LEDSERVICE_SERV_UUID), HI_UINT16(LEDSERVICE_SERV_UUID) };
// ledData UUID
CONST uint8_t ledService_RedUUID[ATT_UUID_SIZE] = {
        TI_BASE_UUID_128(LEDSERVICE_RED_UUID) };
// ledConfig UUID
CONST uint8_t ledService_WhiteUUID[ATT_UUID_SIZE] = {
        TI_BASE_UUID_128(LEDSERVICE_WHITE_UUID) };

/*********************************************************************
 * LOCAL VARIABLES
 */
static ledServiceCBs_t *pAppCBs = NULL;
/*********************************************************************
 * Profile Attributes - variables
 */
// Service declaration
static CONST gattAttrType_t ledServiceDecl = {ATT_BT_UUID_SIZE, ledServiceUUID};

// Characteristic "Red" Properties (for declaration)
static uint8_t ledService_RedProps = GATT_PROP_READ | GATT_PROP_WRITE;
// Characteristic "Red" Value variable
static uint8_t ledService_RedVal = 0;
// Characteristic "Red" User Description
static uint8 ledService_RedDesc[] = "Red LED";

// Characteristic "White" Properties (for declaration)
static uint8_t ledService_WhiteProps = GATT_PROP_READ | GATT_PROP_WRITE;
// Characteristic "White" Value variable
static uint8_t ledService_WhiteVal = 0;
// Characteristic "White" User Description
static uint8 ledService_WhiteDesc[] = "White LED";


/*********************************************************************
 * Profile Attributes - Table
 */
static gattAttribute_t ledServiceAttrTbl[] = {
        // ledService Service Declaration
        {
            {ATT_BT_UUID_SIZE, primaryServiceUUID},
            GATT_PERMIT_READ,
            0,
            (uint8_t *)&ledServiceDecl
        },
        // Red Characteristic Declaration
        {
            {ATT_BT_UUID_SIZE, characterUUID},
            GATT_PERMIT_READ,
            0,
            &ledService_RedProps
        },
        // Red Characteristic Value
        {
            {ATT_UUID_SIZE, ledService_RedUUID},
            GATT_PERMIT_READ | GATT_PERMIT_WRITE,
            0,
            &ledService_RedVal
        },
        // Red Characteristic Value
        {
            { ATT_BT_UUID_SIZE, charUserDescUUID },
            GATT_PERMIT_READ,
            0,
            ledService_RedDesc
        },

        // White Characteristic Declaration
        {
            {ATT_BT_UUID_SIZE, characterUUID},
            GATT_PERMIT_READ,
            0,
            &ledService_WhiteProps
        },
        // White Characteristic Value
        {
            {ATT_UUID_SIZE, ledService_WhiteUUID},
            GATT_PERMIT_READ | GATT_PERMIT_WRITE,
            0,
            &ledService_WhiteVal
        },
        // White Characteristic Value
        {
            { ATT_BT_UUID_SIZE, charUserDescUUID },
            GATT_PERMIT_READ,
            0,
            ledService_WhiteDesc
        }
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t ledService_ReadAttrCB(uint16 connHandle,
                                          gattAttribute_t *pAttr, uint8 *pValue,
                                          uint16 *pLen, uint16 offset,
                                          uint16 maxLen, uint8 method);

static bStatus_t ledService_WriteAttrCB(uint16 connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 len,
                                           uint16 offset, uint8 method);
/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t ledServiceCBs = {
        ledService_ReadAttrCB, // Read callback function pointer
        ledService_WriteAttrCB, // Write callback function pointer
        NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 *
 *
 * LEDService_AddService- Initializes the LEDService service by registering
 *          GATT attributes with the GATT server.
 *
 */
bStatus_t LEDService_AddService(uint32 services) {
    uint8_t status;
    // Register GATT attribute list and CBs with GATT Server App
    if ( services & LEDSERVICE ) {
        status = GATTServApp_RegisterService(ledServiceAttrTbl,
                                             GATT_NUM_ATTRS(ledServiceAttrTbl),
                                             GATT_MAX_ENCRYPT_KEY_SIZE,
                                             &ledServiceCBs);
    } else {
        status = SUCCESS;
    }

    return (status);
}

/*********************************************************************
 * LEDService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t LEDService_RegisterAppCBs(ledServiceCBs_t *appCallbacks) {
    if (appCallbacks) {
        pAppCBs = appCallbacks;
        return (SUCCESS);
    } else {
        return (bleAlreadyInRequestedMode);
    }
}

/****
 * *****************************************************************
 * LEDService_SetParameter - Set a LEDService parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t LEDService_SetParameter(uint8 param, uint8 len, void *value) {
    bStatus_t ret = SUCCESS;
    switch (param) {
        case LEDSERVICE_RED:
            if (len == sizeof ( uint8 )) {
                ledService_RedVal = *((uint8*)value);
            } else {
                ret = bleInvalidRange;
            }
            break;

        case LEDSERVICE_WHITE:
            if ( len == sizeof ( uint8 ) ) {
                ledService_WhiteVal = *((uint8*)value);
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
 * LEDService_GetParameter - Get a LEDService parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
bStatus_t LEDService_GetParameter(uint8 param, void *value) {
    bStatus_t ret = SUCCESS;
    switch (param) {
        case LEDSERVICE_RED:
            *((uint8*)value) = ledService_RedVal;
            break;

        case LEDSERVICE_WHITE:
            *((uint8*)value) = ledService_WhiteVal;
            break;

        default:
            ret = INVALIDPARAMETER;
            break;
    }
    return ret;
}

/*********************************************************************
 * @fn          ledService_ReadAttrCB
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
static bStatus_t ledService_ReadAttrCB(uint16 connHandle,
                                          gattAttribute_t *pAttr, uint8 *pValue,
                                          uint16 *pLen, uint16 offset,
                                          uint16 maxLen, uint8 method) {
    bStatus_t status = SUCCESS;
    // See if request is regarding the Red Characteristic Value
    if (!memcmp(pAttr->type.uuid, ledService_RedUUID, pAttr->type.len)) {
        if (offset > 0) { // Prevent malicious ATT ReadBlob offsets.
            status = ATT_ERR_INVALID_OFFSET;
        } else {
            *pLen = 1;
            pValue[0] = *pAttr->pValue;
        }
    }
    // See if request is regarding the White Characteristic Value
    else if (!memcmp(pAttr->type.uuid, ledService_WhiteUUID, pAttr->type.len)) {
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
    return status;
}

/*********************************************************************
 * @fn      ledService_WriteAttrCB
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
static bStatus_t ledService_WriteAttrCB(uint16 connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 len,
                                           uint16 offset, uint8 method) {
    bStatus_t status = SUCCESS;
    uint8_t paramID = 0xFF;
    // See if request is regarding the Red Characteristic Value
    if (!memcmp(pAttr->type.uuid, ledService_RedUUID, pAttr->type.len)) {
        if (offset + len > 1) {
            status = ATT_ERR_INVALID_OFFSET;
        } else {
            uint8 *pCurValue = (uint8 *)pAttr->pValue;
            *pCurValue = pValue[0];
            // Only notify application if entire expected value is written
            if (offset + len == 1)
                paramID = LEDSERVICE_RED;
        }
    }
    // See if request is regarding the White Characteristic Value
    else if (!memcmp(pAttr->type.uuid, ledService_WhiteUUID, pAttr->type.len)) {
        if (offset + len > 1) {
            status = ATT_ERR_INVALID_OFFSET;
        } else {
            uint8 *pCurValue = (uint8 *)pAttr->pValue;
            *pCurValue = pValue[0];
            // Only notify application if entire expected value is written
            if (offset + len == 1)
                paramID = LEDSERVICE_WHITE;
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
