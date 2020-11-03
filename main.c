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

/** @file
 *
 * @defgroup precure_back_firmware_main main.c
 * @{
 * @ingroup precure_back_firmware
 * @brief Precure Back Firmware for Core Unit.
 *
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "sensorsim.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "ble_conn_state.h"
#include "ble_dis.h"
#include "nrf_ble_bms.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_gpio.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "log_support.h"

#include "our_service.h"
#include "sensor_service.h"
#include "nrf_drv_spi.h"
#include "ads1298_drv.h"

#define DEVICE_NAME                     "Precure Back Core Unit"                /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "Precure A/S"                           /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define SECOND_10_MS_UNITS              100                                     //!< Definition of 1 second, when 1 unit is 10 ms.
#define MIN_CONN_INTERVAL               7                                       //!< Minimum acceptable connection interval (0.25 seconds), Connection interval uses 1.25 ms units.
#define MAX_CONN_INTERVAL               400                                     //!< Maximum acceptable connection interval (0.5 second), Connection interval uses 1.25 ms units.
#define SLAVE_LATENCY                   0                                       //!< Slave latency.
#define CONN_SUP_TIMEOUT                (4 * SECOND_10_MS_UNITS)                //!< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units.

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(15000)                  //!< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds).
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   //!< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds).
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       //!< Number of attempts before giving up the connection parameter negotiation.

#define SEC_PARAM_BOND                  1                                       //!< Perform bonding.
#define SEC_PARAM_MITM                  0                                       //!< Man In The Middle protection not required.
#define SEC_PARAM_LESC                  0                                       //!< LE Secure Connections not enabled.
#define SEC_PARAM_KEYPRESS              0                                       //!< Keypress notifications not enabled.
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    //!< No I/O capabilities.
#define SEC_PARAM_OOB                   0                                       //!< Out Of Band data not available.
#define SEC_PARAM_MIN_KEY_SIZE          7                                       //!< Minimum encryption key size.
#define SEC_PARAM_MAX_KEY_SIZE          16                                      //!< Maximum encryption key size.

#define APP_ADV_FAST_INTERVAL           0x0028                                  //!< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.).
#define APP_ADV_DURATION                18000                                   //!< The advertising duration (180 seconds) in units of 10 milliseconds.

#define MEM_BUFF_SIZE                   512
#define DEAD_BEEF                       0xDEADBEEF                              //!< Value used as error code on stack dump, can be used to identify stack location on stack unwind.
#define SPI_INSTANCE 0

static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);            //!< SPI instance. */
static volatile bool spi_xfer_done;                                             //!< Flag used to indicate that SPI instance completed the transfer. */


APP_TIMER_DEF(m_sec_req_timer_id);                                              //!< Security request timer. The timer lets us start pairing request if one does not arrive from the Central.
NRF_BLE_QWR_DEF(m_qwr);                                                         //!< Context for the Queued Write module.
NRF_BLE_BMS_DEF(m_bms);                                                         //!< Structure used to identify the Bond Management service.
NRF_BLE_GATT_DEF(m_gatt);                                                       //!< GATT module instance.
BLE_ADVERTISING_DEF(m_advertising);                                             //!< Advertising module instance.

static uint16_t                      m_conn_handle = BLE_CONN_HANDLE_INVALID;   //!< Handle of the current connection.
static uint8_t                       m_qwr_mem[MEM_BUFF_SIZE];                  //!< Write buffer for the Queued Write module.
static ble_conn_state_user_flag_id_t m_bms_bonds_to_delete;                     //!< Flags used to identify bonds that should be deleted.

static ble_uuid_t m_adv_uuids[] =                                               //!< Universally unique service identifiers.
{
    {BLE_UUID_BMS_SERVICE,  BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_SENSOR_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN}
};

#ifdef USE_AUTHORIZATION_CODE
static uint8_t m_auth_code[] = {'A', 'B', 'C', 'D'}; //0x41, 0x42, 0x43, 0x44
static int m_auth_code_len = sizeof(m_auth_code);
#endif

ble_os_t m_our_service;
ble_ss_t m_sensor_service;

// app_timer id variable and define our timer interval and define a timer interval
APP_TIMER_DEF(m_our_char_timer_id);
APP_TIMER_DEF(m_sensor_char_timer_id);
#define OUR_CHAR_TIMER_INTERVAL APP_TIMER_TICKS(1000) //1000 ms intervals

// Use UUIDs for service(s) used in your application.
static void advertising_start(bool erase_bonds);

// timer event handler
static void timer_timeout_our_handler(void * p_context)
{
    // Update temperature and characteristic value.
    int32_t temperature = 0;
    static int32_t previous_temperature=0;
    uint32_t err_code = 0;

    sd_temp_get(&temperature);    //Get temperature

    // Check if current temperature is different from last temperature
    if(temperature != previous_temperature)
    {
	// If new temperature then send notification
    }

    // Save current temperature until next measurement
    previous_temperature = temperature;

    nrf_gpio_pin_toggle(LED_4);
}

static uint8_t imu_buf[] = {0x30, 0xe,
			    0x56, 0x56, 0x56, 0x56,
			    0x56, 0x56, 0x56, 0x56,
			    0x56, 0x56, 0x56, 0x56};

static uint8_t emg_buf[] = {0x10, 0x12,
			    0x65, 0x65, 0x65, 0x65,
			    0x65, 0x65, 0x65, 0x65,
			    0x65, 0x65, 0x65, 0x65,
			    0x65, 0x65, 0x65, 0x65};

static void timer_timeout_sensor_handler(void * p_context)
{
    uint32_t err_code = 0;
    static uint8_t buf_index;
    if (buf_index % 2 == 0)
    {
	nrf_gpio_pin_toggle(LED_3);
	err_code = data_stream_update(&m_sensor_service,
				      imu_buf);
    }
    else
    {
	nrf_gpio_pin_toggle(LED_4);
	err_code = data_stream_update(&m_sensor_service,
				      emg_buf);
    }
// Todo:   MY_ERROR_CHECK(err_code);
    MY_ERROR_LOG(err_code);
    buf_index++;
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    MY_ERROR_CHECK(err_code);}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
	delete_bonds();
	// Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
	uint32_t err_code;

	err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
	MY_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling the Security Request timer timeout.
 *
 * @details This function will be called each time the Security Request timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void sec_req_timeout_handler(void * p_context)
{
    ret_code_t              err_code;
    pm_conn_sec_status_t    status;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
	err_code = pm_conn_sec_status_get(m_conn_handle, &status);
        MY_ERROR_CHECK(err_code);

	// If the link is still not secured by the peer, initiate security procedure.
	if (!status.encrypted)
	{
	    err_code = pm_conn_secure(m_conn_handle, false);
	    MY_ERROR_CHECK(err_code);
	}
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
    case PM_EVT_BONDED_PEER_CONNECTED:
    {
	NRF_LOG_INFO("Connected to a previously bonded device.");
    } break;

    case PM_EVT_CONN_SEC_SUCCEEDED:
    {
	NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
		     ble_conn_state_role(p_evt->conn_handle),
		     p_evt->conn_handle,
		     p_evt->params.conn_sec_succeeded.procedure);
    } break;

    case PM_EVT_CONN_SEC_FAILED:
    {
	/* Often, when securing fails, it shouldn't be restarted, for security reasons.
	 * Other times, it can be restarted directly.
	 * Sometimes it can be restarted, but only after changing some Security Parameters.
	 * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
	 * Sometimes it is impossible, to secure the link, or the peer device does not support it.
	 * How to handle this error is highly application dependent. */
    } break;

    case PM_EVT_CONN_SEC_CONFIG_REQ:
    {
	// Reject pairing request from an already bonded peer.
	pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
	pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
    } break;

    case PM_EVT_STORAGE_FULL:
    {
	// Run garbage collection on the flash.
	err_code = fds_gc();
	if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
	{
	    // Retry.
	}
	else
	{
	    MY_ERROR_CHECK(err_code);
	}
    } break;

    case PM_EVT_PEERS_DELETE_SUCCEEDED:
    {
	advertising_start(false);
    } break;

    case PM_EVT_PEER_DATA_UPDATE_FAILED:
    {
	APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
    } break;

    case PM_EVT_PEER_DELETE_FAILED:
    {
	// Assert.
	APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
    } break;

    case PM_EVT_PEERS_DELETE_FAILED:
    {
	// Assert.
	APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
    } break;

    case PM_EVT_ERROR_UNEXPECTED:
    {
	// Assert.
	APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
    } break;

    case PM_EVT_CONN_SEC_START:
    case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
    case PM_EVT_PEER_DELETE_SUCCEEDED:
    case PM_EVT_LOCAL_DB_CACHE_APPLIED:
    case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
	// This can happen when the local DB has changed.
    case PM_EVT_SERVICE_CHANGED_IND_SENT:
    case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
    default:
	break;
    }
}


/**@brief Function for marking the requester's bond for deletion.
 */
static void delete_requesting_bond(nrf_ble_bms_t const * p_bms)
{
    NRF_LOG_INFO("Client requested that bond to current device deleted");
    ble_conn_state_user_flag_set(p_bms->conn_handle, m_bms_bonds_to_delete, true);
}


/**@brief Function for deleting a single bond if it does not belong to a connected peer.
 *
 * This will mark the bond for deferred deletion if the peer is connected.
 */
static void bond_delete(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t   err_code;
    pm_peer_id_t peer_id;

    if (ble_conn_state_status(conn_handle) == BLE_CONN_STATUS_CONNECTED)
    {
	ble_conn_state_user_flag_set(conn_handle, m_bms_bonds_to_delete, true);
    }
    else
    {
	NRF_LOG_DEBUG("Attempting to delete bond.");
	err_code = pm_peer_id_get(conn_handle, &peer_id);
	if (err_code == NRF_SUCCESS)
	{
	    err_code = pm_peer_delete(peer_id);
	    MY_ERROR_CHECK(err_code);
	    ble_conn_state_user_flag_set(conn_handle, m_bms_bonds_to_delete, false);
	}
    }
}


/**@brief Function for deleting all bonds
 */
static void delete_all_bonds(nrf_ble_bms_t const * p_bms)
{
    ret_code_t err_code;
    uint16_t conn_handle;

    NRF_LOG_INFO("Client requested that all bonds be deleted");

    pm_peer_id_t peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    while (peer_id != PM_PEER_ID_INVALID)
    {
	err_code = pm_conn_handle_get(peer_id, &conn_handle);
	MY_ERROR_CHECK(err_code);

	bond_delete(conn_handle, NULL);

	peer_id = pm_next_peer_id_get(peer_id);
    }
}



/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    MY_ERROR_CHECK(err_code);

    // Initiate timer
    app_timer_create(&m_our_char_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_our_handler);
    app_timer_create(&m_sensor_char_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_sensor_handler);

}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    MY_ERROR_CHECK(err_code);
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    MY_ERROR_CHECK(err_code);}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    MY_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from bond management service.
 */
void bms_evt_handler(nrf_ble_bms_t * p_ess, nrf_ble_bms_evt_t * p_evt)
{
    ret_code_t err_code;
    bool is_authorized = true;

    switch (p_evt->evt_type)
    {
    case NRF_BLE_BMS_EVT_AUTH:
	NRF_LOG_DEBUG("Authorization request.");
#if USE_AUTHORIZATION_CODE
	if ((p_evt->auth_code.len != m_auth_code_len) ||
	    (memcmp(m_auth_code, p_evt->auth_code.code, m_auth_code_len) != 0))
	{
	    is_authorized = false;
	}
#endif
	err_code = nrf_ble_bms_auth_response(&m_bms, is_authorized);
	MY_ERROR_CHECK(err_code);
    }
}


/**@brief Function for deleting all bet requesting device bonds
 */
static void delete_all_except_requesting_bond(nrf_ble_bms_t const * p_bms)
{
    ret_code_t err_code;
    uint16_t conn_handle;

    NRF_LOG_INFO("Client requested that all bonds except current bond be deleted");

    pm_peer_id_t peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    while (peer_id != PM_PEER_ID_INVALID)
    {
	err_code = pm_conn_handle_get(peer_id, &conn_handle);
	MY_ERROR_CHECK(err_code);

	/* Do nothing if this is our own bond. */
	if (conn_handle != p_bms->conn_handle)
	{
	    bond_delete(conn_handle, NULL);
	}

	peer_id = pm_next_peer_id_get(peer_id);
    }
}


/**@brief Function that Initialize the Bond Management Service
 */
static void bms_init(void)
{
    ret_code_t           err_code;
    nrf_ble_bms_init_t   bms_init;

    // 
    memset(&bms_init, 0, sizeof(bms_init));

    m_bms_bonds_to_delete        = ble_conn_state_user_flag_acquire();
    bms_init.evt_handler         = bms_evt_handler;
    bms_init.error_handler       = service_error_handler;
#if USE_AUTHORIZATION_CODE
    bms_init.feature.delete_requesting_auth         = true;
    bms_init.feature.delete_all_auth                = true;
    bms_init.feature.delete_all_but_requesting_auth = true;
#else
    bms_init.feature.delete_requesting              = true;
    bms_init.feature.delete_all                     = true;
    bms_init.feature.delete_all_but_requesting      = true;
#endif
    bms_init.bms_feature_sec_req = SEC_JUST_WORKS;
    bms_init.bms_ctrlpt_sec_req  = SEC_JUST_WORKS;

    bms_init.p_qwr                                       = &m_qwr;
    bms_init.bond_callbacks.delete_requesting            = delete_requesting_bond;
    bms_init.bond_callbacks.delete_all                   = delete_all_bonds;
    bms_init.bond_callbacks.delete_all_except_requesting = delete_all_except_requesting_bond;

    err_code = nrf_ble_bms_init(&m_bms, &bms_init);
    MY_ERROR_CHECK(err_code);
}

uint16_t qwr_evt_handler(nrf_ble_qwr_t * p_qwr, nrf_ble_qwr_evt_t * p_evt)
{
    return nrf_ble_bms_on_qwr_evt(&m_bms, p_qwr, p_evt);
}

/**@brief Function that initialise the Initialize Queued Write Module
 */
static void qwr_init(void)
{
    ret_code_t           err_code;
    nrf_ble_qwr_init_t   qwr_init;

    memset(&qwr_init, 0, sizeof(qwr_init));
    qwr_init.mem_buffer.len   = MEM_BUFF_SIZE;
    qwr_init.mem_buffer.p_mem = m_qwr_mem;
    qwr_init.callback         = qwr_evt_handler;
    qwr_init.error_handler    = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    MY_ERROR_CHECK(err_code);
}

/**@brief Initialise the Device Information Service
 */
static void dis_init(void)
{
    ret_code_t err_code;
    ble_dis_init_t dis_init;

    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str,
			  MANUFACTURER_NAME);

    err_code = ble_dis_init(&dis_init);
    MY_ERROR_CHECK(err_code);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t         err_code;

    qwr_init();
    bms_init();
    dis_init();

    // Add code to initialize the services used by the application.
    our_service_init(&m_our_service);
    sensor_service_init(&m_sensor_service);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
	err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
	MY_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    MY_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{

    //app_timer_start(m_our_char_timer_id, OUR_CHAR_TIMER_INTERVAL, NULL);
    //To update temperature only when in a connection then don't
    //call app_timer_start() here, but in ble_event_handler()
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    MY_ERROR_CHECK(err_code);
    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    MY_ERROR_CHECK(err_code);
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
// Todo:    err_code = sd_power_system_off();
    MY_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
    case BLE_ADV_EVT_FAST:
	NRF_LOG_INFO("Fast advertising.");
	err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
	MY_ERROR_CHECK(err_code);
	break;

    case BLE_ADV_EVT_IDLE:
	sleep_mode_enter();
	break;

    default:
	break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;
		

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_DISCONNECTED:
	NRF_LOG_INFO("Disconnected.");
	// LED indication will be changed when advertising starts.
 

	break;

    case BLE_GAP_EVT_CONNECTED:
	NRF_LOG_INFO("Connected.");
	err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
	MY_ERROR_CHECK(err_code);
	m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
	err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
	MY_ERROR_CHECK(err_code);

	//When connected; start our timer to start regular temperature measurements
	app_timer_start(m_our_char_timer_id, OUR_CHAR_TIMER_INTERVAL, NULL);
	app_timer_start(m_sensor_char_timer_id, OUR_CHAR_TIMER_INTERVAL, NULL);
	break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
	NRF_LOG_DEBUG("PHY update request.");
	ble_gap_phys_t const phys =
	    {
		.rx_phys = BLE_GAP_PHY_AUTO,
		.tx_phys = BLE_GAP_PHY_AUTO,
	    };
	err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
	MY_ERROR_CHECK(err_code);
    } break;

    case BLE_GATTC_EVT_TIMEOUT:
	// Disconnect on GATT Client timeout event.
	NRF_LOG_DEBUG("GATT Client Timeout.");
	err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
					 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	MY_ERROR_CHECK(err_code);
	break;

    case BLE_GATTS_EVT_TIMEOUT:
	// Disconnect on GATT Server timeout event.
	NRF_LOG_DEBUG("GATT Server Timeout.");
	err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
					 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	MY_ERROR_CHECK(err_code);
	break;

    default:
	// No implementation needed.
	break;
    }

		
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    MY_ERROR_CHECK(err_code);
    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    MY_ERROR_CHECK(err_code);
    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    MY_ERROR_CHECK(err_code);
    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

    // Set up a BLE event observer to call ble_sensor_service_on_ble_evt() to do housekeeping of ble connections related to sensor service and characteristics.
    NRF_SDH_BLE_OBSERVER(m_our_service_observer, APP_BLE_OBSERVER_PRIO, ble_our_service_on_ble_evt, (void*) &m_our_service);
	
    // Set up a BLE event observer to call ble_sensor_service_on_ble_evt() to do housekeeping of ble connections related to sensor service and characteristics.
    NRF_SDH_BLE_OBSERVER(m_sensor_service_observer, APP_BLE_OBSERVER_PRIO, ble_sensor_service_on_ble_evt, (void*) &m_sensor_service);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    MY_ERROR_CHECK(err_code);
    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    MY_ERROR_CHECK(err_code);
    err_code = pm_register(pm_evt_handler);
    MY_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
    case BSP_EVENT_SLEEP:
	sleep_mode_enter();
	break; // BSP_EVENT_SLEEP

    case BSP_EVENT_DISCONNECT:
	err_code = sd_ble_gap_disconnect(m_conn_handle,
					 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	if (err_code != NRF_ERROR_INVALID_STATE)
	{
	    MY_ERROR_CHECK(err_code);
	}
	break; // BSP_EVENT_DISCONNECT

    case BSP_EVENT_WHITELIST_OFF:
	if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
	{
	    err_code = ble_advertising_restart_without_whitelist(&m_advertising);
	    if (err_code != NRF_ERROR_INVALID_STATE)
	    {
		MY_ERROR_CHECK(err_code);
	    }
	}
	break; // BSP_EVENT_KEY_0

    default:
	break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

	
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids = m_adv_uuids;
		
	
    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    MY_ERROR_CHECK(err_code);
    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    MY_ERROR_CHECK(err_code);
    err_code = bsp_btn_ble_init(NULL, &startup_event);
    MY_ERROR_CHECK(err_code);
    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    MY_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    MY_ERROR_CHECK(nrf_pwr_mgmt_init());
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
	nrf_pwr_mgmt_run();
    }
}


/**@brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;
    bool erase_bonds;

    // Initialize.
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();

    bsp_board_init(BSP_INIT_LEDS);
    (void)ads_init_spi();
    err_code = NRF_LOG_INIT(NULL);
    MY_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Precure Back Firmware started.");

    application_timers_start();

    advertising_start(erase_bonds);

//    (void)ads_read_ID();
    (void)ads_hello_world();
   
    for (;;)
    {
	idle_state_handle();

        /* bsp_board_led_invert(BSP_BOARD_LED_0); */
	nrf_gpio_pin_toggle(SPI_SS_PIN);     // SPI chip select; active low
    }
}


/**
 * @}
 */
