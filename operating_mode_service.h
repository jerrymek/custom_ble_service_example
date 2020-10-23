/**
 * Copyright  2020 PreCure ApS
 *
 * All Rights Reserved
 */
#ifndef OPERATING_MODE_SERVICE_H__
#define OPERATING_MODE_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "uuid.h"

/* This structure contains various status information for
 * the service Operating Mode. 
 */
typedef struct
{
    uint16_t                 conn_handle;    /**< Handle of the current connection. */
    uint16_t                 service_handle; /**< Handle of the service Operating Mode. */
    uint16_t                 system_state;   /**< Handle of the characteristic System State. */
    ble_gatts_char_handles_t char_handles;
} ble_om_t;

/* This structure contains various status information for
 * the Operating Mode Service . 
 */
typedef struct
{
    uint16_t                 conn_handle;
    uint16_t                 service_handle;
    uint16_t                 data_stream; // 
    uint16_t                 imu_sensor_frequency; 
    uint16_t                 emg_sensor_frequency; 
    uint16_t                 emg_filter_frequency; 
    uint16_t                 motion_sensitivity; 
    ble_gatts_char_handles_t char_handles;
} ble_ss_t;

/* This structure contains various status information for
 * the Operating Mode (service). 
 */
typedef struct
{
    uint16_t                 conn_handle;
    uint16_t                 service_handle;
    uint16_t                 configuration; 
    uint16_t                 command; 
    ble_gatts_char_handles_t char_handles;
} ble_cs_t;

/**@brief Function for handling BLE Stack events related to operating mode service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Operating Mode Service.
 *
 * @param[in]   p_operating_mode_service         Operating Mode Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_operating_mode_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_operating_mode_service         Pointer to Operating Mode Service structure.
 */
void operating_mode_service_init(ble_ss_t * p_operating_mode_service);

#endif  /* OPERATING_MODE_SERVICE_H__ */
