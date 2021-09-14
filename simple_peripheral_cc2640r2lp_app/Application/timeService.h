#ifndef APPLICATION_TIMESYNCSERVICE_H_
#define APPLICATION_TIMESYNCSERVICE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "bcomdef.h"

//  Characteristic defines
#define TIMESERVICE_SYNC_DATA 0
#define TIMESERVICE_WRITE_DATA 1

#define TIMESERVICE_DATA_LEN 10

#define TIMESERVICE_WRITE_FLAG 2

typedef void (*timeServiceChange_t)();

typedef struct
{
  timeServiceChange_t pfnChangeCb;  // Called when characteristic value changes
} timeServiceCBs_t;


/*********************************************************************
 * API FUNCTIONS
 */


/*
 * TimeService_AddService- Initializes the TimeService service by registering
 *          GATT attributes with the GATT server.
 *
 */
extern bStatus_t TimeService_AddService( void );

/*
 * TimeService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t TimeService_RegisterAppCBs( timeServiceCBs_t *appCallbacks );

/*
 * TimeService_SetParameter - Set a TimeService parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t TimeService_SetParameter( uint8 param, uint8 len, void *value );

/*
 * TimeService_GetParameter - Get a TimeService parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to read.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t TimeService_GetParameter( uint8 param, void *value );

/*********************************************************************
*********************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_TIMESYNCSERVICE_H_ */
