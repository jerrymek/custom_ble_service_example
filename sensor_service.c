/**
 * Copyright  2020 PreCure ApS
 *
 * All Rights Reserved
 */
/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "sensor_service.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "log_support.h"

// Declaration of a function that will take care of some housekeeping of ble
// connections related to the Sensor Service and its characteristics.
void ble_sensor_service_on_ble_evt(ble_evt_t const * p_ble_evt,
				   void * p_context)
{
    // Implement switch case handling BLE events related to sensor service. 
    ble_ss_t * p_sensor_service =(ble_ss_t *) p_context;
    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        p_sensor_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        break;
    case BLE_GAP_EVT_DISCONNECTED:
        p_sensor_service->conn_handle = BLE_CONN_HANDLE_INVALID;
        break;
    default:
        break;
    }
}

/**@brief Function for adding a new characterstic to the sensor service. 
 *
 * @param[in]   p_sensor_service        Sensor Service structure.
 *
 */
static uint32_t sensor_char_add(ble_ss_t * p_sensor_service)
{
    // Add a custom characteristic UUID
    uint32_t            err_code;
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_PRECURE_BACK_BASE_UUID;
    char_uuid.uuid      = BLE_UUID_SENSOR_SERVICE_DATA_STREAM_UUID;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    MY_ERROR_CHECK(err_code);  
    
    // Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    
    // Configuring Client Characteristic Configuration Descriptor metadata and
    // add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc              = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md         = &cccd_md;
    char_md.char_props.notify = 1;
    
    // Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));  
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;
    
    // Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    // Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    
    // Set characteristic length in number of bytes
    attr_char_value.max_len     = 0x12;
    attr_char_value.init_len    = 2;
    uint8_t value[4]            = {0x30, 2};
    attr_char_value.p_value     = value;

    // Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_sensor_service->service_handle,
					       &char_md,
					       &attr_char_value,
					       &p_sensor_service->char_handles);
    MY_ERROR_CHECK(err_code);
    return NRF_SUCCESS;
}


/**@brief Function for initiating our new service.
 *
 * @param[in]   p_sensor_service        Sensor Service structure.
 *
 */
void sensor_service_init(ble_ss_t * p_sensor_service)
{
    uint32_t   err_code;

    // Declare 16-bit service and 128-bit base UUIDs and add them
    // to the BLE stack
    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_PRECURE_BACK_BASE_UUID;
    service_uuid.uuid = BLE_UUID_SENSOR_SERVICE_UUID;
    service_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    MY_ERROR_CHECK(err_code);    
    
    // Set sensor service connection handle to default value.
    // I.e. an invalid handle since we are not yet in a connection.
    p_sensor_service->conn_handle = BLE_CONN_HANDLE_INVALID;

    // sensor service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_sensor_service->service_handle);
    MY_ERROR_CHECK(err_code);

    uint32_t flags;
    /* sd_ble_gatts_sys_attr_set(uint16_t conn_handle, */
    /*                           uint8_t const *p_sys_attr_data, */
    /*                           sizeof(p_sensor_service), */
    /*                           uint32_t flags)); */
    err_code = sd_ble_gatts_sys_attr_set(p_sensor_service->conn_handle, NULL, 0, flags);
    MY_ERROR_CHECK(err_code);

    // Call the function sensor_char_add() to add our new characteristic
    // to the service. 
    sensor_char_add(p_sensor_service);
}

uint32_t data_stream_update(ble_ss_t *p_sensor_service,
                            uint8_t *buf)
{
    uint32_t err_code = 0;
    if (p_sensor_service->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));
        hvx_params.handle = p_sensor_service->char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = buf[1];
        hvx_params.p_data = buf;

        err_code = sd_ble_gatts_hvx(p_sensor_service->conn_handle, &hvx_params);
        MY_ERROR_CHECK(err_code);
    }
    return err_code;
}

/* eof */
