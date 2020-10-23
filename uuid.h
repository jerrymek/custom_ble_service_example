/**
 * Copyright  2020 PreCure ApS
 *
 * All Rights Reserved
 */
#ifndef UUID_H__
#define UUID_H__

// 6cc97305-b908-4dc9-8a1a-9fd001a8b75c
// generated by https://www.uuidgenerator.net/version4
// See https://www.ietf.org/rfc/rfc4122.txt
// Defining 16-bit service and 128-bit base UUIDs
// 128-bit base UUID
#define BLE_UUID_PRECURE_BACK_BASE_UUID                   {{0X5C, 0XB7, 0XA8, 0X01, 0XD0, 0X9F, 0X1A, 0X8A,\
							    0XC9, 0X4D, 0X08, 0XB9, 0X05, 0X73, 0XC9, 0X6C}}
#define BLE_UUID_OPERATING_MODE_UUID                      0xF000
#define BLE_UUID_OPERATING_MODE_SYSTEM_STATE_UUID         0xF001
#define BLE_UUID_SENSOR_SERVICE_UUID                      0xE000
#define BLE_UUID_SENSOR_SERVICE_DATA_STREAM_UUID          0xE000
#define BLE_UUID_SENSOR_SERVICE_SENSOR_STATE_UUID         0xE002
#define BLE_UUID_SENSOR_SERVICE_IMU_SENSOR_FREQUENCY_UUID 0xE003
#define BLE_UUID_SENSOR_SERVICE_EMG_SENSOR_FREQUENCY_UUID 0xE004
#define BLE_UUID_SENSOR_SERVICE_EMG_FILTER_FREQUENCY_UUID 0xE005
#define BLE_UUID_SENSOR_SERVICE_MOTION_SENSITIVITY_UUID   0xE006
#define BLE_UUID_VIBRATOR_SERVICE_UUID                    0xD000
#define BLE_UUID_VIBRATOR_SERVICE_VIBRATOR_MODE_UUID      0xD001
#define BLE_UUID_CONFIGURATION_SERVICE_UUID               0xC000
#define BLE_UUID_CONFIGURATION_SERVICE_COMMAND_UUID       (0xC001)

#endif // UUID_H__
         