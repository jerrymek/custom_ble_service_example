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
#include "uuid.h"

/* This structure contains various status information for
 * the service Operating Mode. 
 */
typedef struct
{
    uint16_t                 conn_handle;                 /**< Handle of the current connection. */
    uint16_t                 service_handle;              /**< Handle of the Sensor Service. */
    uint16_t                 data_stream_handle;          /**< Handle of the Characteristic Data Stream. */
    uint16_t                 sensor_state_handle;         /**< Handle of the Characteristic Sensor State. */
    uint16_t                 imu_sensor_frequency_handle; /**< Handle of the Characteristic Sensor State. */
    uint16_t                 emg_sensor_frequency_handle; /**< Handle of the Characteristic Sensor State. */
    uint16_t                 emg_filter_frequency_handle; /**< Handle of the Characteristic Sensor State. */
    uint16_t                 motion_sensitivity_handle;   /**< Handle of the Characteristic Sensor State. */
    ble_gatts_char_handles_t char_handles;
} ble_ss_t;

/**
 * @brief Definitions of the sensor ID
 */
#define emg_data_1  0x10
#define emg_data_2  0x11
#define emg_data_3  0x12
#define emg_data_4  0x13
#define emg_info_1  0x20
#define emg_info_2  0x21
#define emg_info_3  0x22
#define emg_info_4  0x23
#define imu_acc_1   0x30
#define imu_acc_2   0x31
#define imu_acc_3   0x32
#define imu_mag_1   0x40
#define imu_mag_2   0x41
#define imu_mag_3   0x42
#define imu_gyro_1  0x50
#define imu_gyro_2  0x51
#define imu_gyro_3  0x52
#define imu_euler_1 0x60
#define imu_euler_2 0x61
#define imu_euler_3 0x62

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
    uint8_t sensor_id;
    uint8_t length;
    void *data;
} data_stream_packet_t;

/**
 * @brief Format of sensor data sent as part of the data stream packet
 * @detail The sensor data has the same structure: x, y and z of the
 *         type float for all IMU sensor types accelerometer,
 *         magnetometer and gyroscope.
 */
typedef struct
{
    float x_axis;
    float y_axis;
    float z_axis;
} imu_data_t;

/**
 * @brief Format of euler angle data of sent as part of the data stream
 * @detail The sensor data has the structure: alpha, beta and gamma, and
 *         have the type float.
 */
typedef struct
{
    float alpha;
    float beta;
    float gamma;
} euler_angels_t;

/**
 * @brief Pointer to buffer with sensor data
 * @details The buffer can be of the types
 */
//uint8_t *buf_p;

/**
 * @brief Get sensor data
 * @details Get sensor data from the sensor specified by sensor_id_enum.
 *          If successful data is returned in the buffer
 *          The function return 0 if successful.
 */
static uint8_t get_sensor_data(uint8_t sensor_id,
			       uint8_t *buf_p);

/**
 * @brief Get sensor data from the HW
 * @details Get sensor data from the sensor specified by sensor_id_enum.
 *          Data is returned in the buffer pointed to by buf_p.
 *          The number of packets in the buffer is specified by length.
 *          The function return 0 if successful.
 */
uint32_t data_stream_update(ble_ss_t *p_sensor_service,
			    uint8_t *buf);

/**
 * @brief Initiate the IMU sensor HW
 * @details Do neccessery register setup and load firmaware that runs
 *          the Sensor Fusion HW support.
 *          Setup the GPIO pins.
 *          The function return 0 if successful.
 */
uint8_t initiate_imu_hw(void /* TBD */);

#endif  /* SENSOR_SERVICE_H__ */
