/**
 * Copyright  2020 PreCure ApS
 *
 * All Rights Reserved
 */
#ifndef SENSOR_SERVICE_H__
#define SENSOR_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "imu_hw_init.h"
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

/**
 * @brief Enum for the sensor ID
 * @details Definition of values for the available sensors.
 */
typedef enum
{
    emg_data_1 = 0x10,
    emg_data_2 = 0x11,
    emg_data_3 = 0x12,
    emg_data_4 = 0x13,
    emg_info_1 = 0x20,
    emg_info_2 = 0x21,
    emg_info_3 = 0x22,
    emg_info_4 = 0x23,
    imu_acc_1  = 0x30,
    imu_acc_2  = 0x31,
    imu_acc_3  = 0x32,
    imu_mag_1  = 0x40,
    imu_mag_2  = 0x41,
    imu_mag_3  = 0x42,
    imu_gyro_1  = 0x50,
    imu_gyro_2  = 0x51,
    imu_gyro_3  = 0x52,
    imu_euler_1  = 0x60,
    imu_euler_2  = 0x61,
    imu_euler_3  = 0x62
} sensor_id_enum;
    
/* This structure contains various status information for
 * the Sensor Service . 
 */
typedef struct
{
    uint16_t                 conn_handle;
    uint16_t                 service_handle;
    sensor_id_enum           data_stream;
    uint16_t                 imu_sensor_frequency; 
    uint16_t                 emg_sensor_frequency; 
    uint16_t                 emg_filter_frequency; 
    uint16_t                 motion_sensitivity; 
    ble_gatts_char_handles_t char_handles;
} ble_ss_t;

/**@brief Function for handling BLE Stack events related to sensor service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Sensor Service.
 *
 * @param[in]   p_sensor_service         Sensor Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_sensor_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_sensor_service         Pointer to Sensor Service structure.
 */
void sensor_service_init(ble_ss_t * p_sensor_service);

typedef struct
{
    sensor_id_enum sensor_id;
    uint8_t length;
    uint8_t *data;
} data_stream_packet_t;

/**
 * @brief Format of sensor data sent as part of the data stream packet
 * @detail The sensor data has the same structure: x, y and z of the
 *         type float for all sensor types accelerometer, magnetometer
 *         and gyroscope.
 */
typedef struct
{
    sensor_id_enum sensor_id;
    float x_axis;
    float y_axis;
    float z_axis;
} sensor_data_t;

/**
 * @brief Format of euler angle data of sent as part of the data stream
 * @detail  packet .The sensor data has the same structure: x, y and z
 *         of the type float for all sensor types accelerometer,
 *         magnetometer and gyroscope.
 */
typedef struct
{
    sensor_id_enum sensor_id;
    float alpha;
    float beta;
    float gamma;
} sensor_euler_t;

/**
 * @brief Get sensor data from the HW
 * @details Get sensor data from the sensor specified by sensor_id_enum.
 *          If successful data is returned in the buffer
 *          The function return 0 if successful.
 */
static uint8_t get_sensor_data_from_hw(sensor_id_enum sensor_id,
				       sensor_data_t *buf_t);

/**
 * @brief Get sensor data from the HW
 * @details Get sensor data from the sensor specified by sensor_id_enum.
 *          Data is returned in the buffer pointed to by buf_p.
 *          The number of packets in the buffer is specified by length.
 *          The function return 0 if successful.
 */
uint8_t send_sensor_data_to_mobile(sensor_id_enum sensor_id,
				   uint8_t length,
				   data_stream_packet_t *buf_t);

/**
 * @brief Initiate the IMU sensor HW
 * @details Do neccessery register setup and load firmaware that runs
 *          the Sensor Fusion HW support.
 *          Setup the GPIO pins.
 *          The function return 0 if successful.
 */
uint8_t initiate_imu_hw(void /* TBD */);

#endif  /* SENSOR_SERVICE_H__ */
