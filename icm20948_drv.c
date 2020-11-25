/**
 * Copyright  2020 PreCure ApS
 *
 * All Rights Reserved
 */
/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
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
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_twim.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "log_support.h"
#include "icm20948_drv.h"

/* TWI instance ID. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(1);

static uint8_t CurrentChannel;

typedef struct
{
    uint8_t  device_id;
    uint8_t  packet_len;
    union
    {
	float f;
	uint16_t u;
    } acc_x;
    union
    {
	float f;
	uint16_t u;
    } acc_y;
    union
    {
	float f;
	uint16_t u;
    } acc_z;
    union
    {
	float f;
	uint16_t u;
    } gyr_x;
    union
    {
	float f;
	uint16_t u;
    } gyr_y;
    union
    {
	float f;
	uint16_t u;
    } gyr_z;
    union
    {
	float f;
	uint16_t u;
    } mag_x;
    union
    {
	float f;
	uint16_t u;
    } mag_y;
    union
    {
	float f;
	uint16_t u;
    } mag_z;
} imu_raw_data_t;

imu_raw_data_t imuRawData;

/**
 * 0x69 is the ICM20948's address default I2C address.
 * 68 is also possible if need
 */
#define IMU11_ADDR       (0x69U) // Todo: >> 1) // Channel 1, device 1
#define IMU12_ADDR       (0x68U) // Todo: >> 1) // Channel 1, device 2
#define IMU21_ADDR       (0x69U) // Todo: >> 1) // Channel 2, device 1

uint8_t imu_addr[3] =
{
    0x69, // I2C bus 1
    0x69, // Todo: 0x68,
    0x69  // I2C bus 2
};

#define IMU_MAG_ADDR     (0x0C)

void icm_get_imu_data(icm_imu_data_t *imu_data)
{
    imu_data->device_id = imuRawData.device_id;
    imu_data->packet_length = imuRawData.packet_len;
    imu_data->acc_x.u = imuRawData.acc_x.u;
    imu_data->acc_y.u = imuRawData.acc_y.u;
    imu_data->acc_z.u = imuRawData.acc_z.u;
    NRF_LOG_DEBUG("0x%x, %d, Acceleromet2  int32 = %d, %d, %d\n",
		  imu_data->device_id, imu_data->packet_length,
		  imu_data->acc_x.u, imu_data->acc_y.u, imu_data->acc_z.u);

    /* Data from the gyroscope
     * in the ICM chip.
     */
    imu_data->gyr_x.u = imuRawData.gyr_x.u;
    imu_data->gyr_y.u = imuRawData.gyr_y.u;
    imu_data->gyr_z.u = imuRawData.gyr_z.u;

    /* Data from the magnetometer
     * in the ICM chip.
     */
    imu_data->mag_x.u = imuRawData.mag_x.u;
    imu_data->mag_y.u = imuRawData.mag_y.u;
    imu_data->mag_z.u = imuRawData.mag_z.u;
}

static void gpio_cfg(uint32_t pin, uint32_t cfg)
{
    NRF_GPIO_Type * reg = nrf_gpio_pin_port_decode(&pin);
    reg->PIN_CNF[pin] = cfg;	
}

static void gpio_init(void)
{
    gpio_cfg(I2C_SDA1_PIN,0x0000060C);
    gpio_cfg(I2C_SCL1_PIN,0x0000060C);
}

ret_code_t io_i2cInit()
{
    ret_code_t err_code = GENERAL_FAILURE;

    // default Channel 0
    const nrf_drv_twi_config_t twi_imu_config =
	{
	    I2C_SCL1_PIN,
	    I2C_SDA1_PIN,
	    NRF_DRV_TWI_FREQ_100K,
	    APP_IRQ_PRIORITY_HIGH,
	    false,
	    false
	};
	
    err_code = nrf_drv_twi_init(&m_twi,
				&twi_imu_config,
				NULL/*no handler*/,
				NULL);
    MY_ERROR_LOG(err_code)
	
    nrf_drv_twi_enable(&m_twi);

    return err_code;
}

void io_i2cSetChannel(uint8_t Channel)
{
    if(Channel == CurrentChannel)
	return;

    CurrentChannel = Channel;

    NRF_TWIM_Type * p_twim = m_twi.u.twim.p_twim;
	
    nrf_twim_pins_set(p_twim,
		      I2C_SDA1_PIN,
		      I2C_SCL1_PIN);
}

nrfx_err_t io_i2cTx(uint8_t Address, char *data, uint16_t Len, uint8_t noStop)
{
    return nrf_drv_twi_tx(&m_twi, Address, data, Len, noStop);
}

nrfx_err_t io_i2cRx(uint8_t Address, char *dest, uint16_t Len)
{
    return nrf_drv_twi_rx(&m_twi, Address, dest, Len);
}

void icmInitI2c(void)
{
    MY_ERROR_CHECK(io_i2cInit()); 
    io_i2cSetChannel(0);
}

void icmDeviceReset(uint8_t imu_number)
{
    char reg_addr = ICM_PWR_MGMT_1;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, (ICM_PWR_MGMT_1_DEVICE_RESET | ~ICM_PWR_MGMT_1_TEMP_DIS)));
    nrf_delay_ms(200);
}

extern void icmReadChipId(uint8_t imu_number)
{
    char reg_addr = ICM_WHO_AM_I;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, TX_NO_STOP));
    char result[1] = {0};
    MY_ERROR_CHECK(io_i2cRx(imu_addr[imu_number], result, 1));
    NRF_LOG_DEBUG("%s(%d) Chip ID = 0x%x", __FILENAME__, __LINE__, *result);
}

void icmInitiateIcm20948(uint8_t imu_number)
{
    char data[2] = {ICM_PWR_MGMT_1, 0x01};
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], data, 2, TX_NO_STOP));
    data[0] = ICM_PWR_MGMT_2;
    data[1] = 0x00;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], data, 2, TX_NO_STOP));

    data[0] = ICM_REG_BANK_SEL;
    data[1] = ICM_USER_BANK_2;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], data, 2, TX_NO_STOP));

    data[0] = ICM_GYRO_CONFIG_1;
    data[1] = 0x19;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], data, 2, TX_NO_STOP));

    data[0] = ICM_TEMP_CONFIG;
    data[1] = 0x03;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], data, 2, TX_NO_STOP));

    data[0] = ICM_GYRO_SMPLRT_DIV;
    data[1] = 0x04;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], data, 2, TX_NO_STOP));

    data[0] = ICM_ACCEL_CONFIG;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], data, 1, TX_NO_STOP));
    uint8_t c = 0;
    MY_ERROR_CHECK(io_i2cRx(imu_addr[imu_number], &c, 1));
    c = c & ~0x06;
    c = c | 0x00 << 1;
    c = c | 0x01;
    c = c | 0x18;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &c, 1, TX_NO_STOP));

    data[0] = ICM_ACCEL_SMPLRT_DIV_1;
    data[1] = 0;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], data, 2, TX_NO_STOP));
    data[0] = ICM_ACCEL_SMPLRT_DIV_2;
    data[1] = 5;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], data, 2, TX_NO_STOP));

    data[0] = ICM_ACCEL_CONFIG;
    data[1] = 1 | (1 << 1) | (3 << 3);
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], data, 1, TX_NO_STOP));
    uint16_t acc_scale = 4.0 / (1 << 15);
    uint16_t gyro_scale = 500.0 / (1 << 15);

    data[0] = ICM_REG_BANK_SEL;
    data[1] = ICM_USER_BANK_0;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], data, 2, TX_NO_STOP));

    data[0] = ICM_INT_PIN_CFG;
    data[1] = 0x22;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], data, 2, TX_NO_STOP));

    data[0] = ICM_INT_ENABLE_1;
    data[1] = 0x01;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], data, 2, TX_NO_STOP));
}

void readMagnReg (uint8_t imu_number, uint8_t reg, uint8_t length)
{
    char reg_addr = ICM_REG_BANK_SEL;
    printf ("0x%x, 0x%x\n", imu_addr[imu_number], imu_number);
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, ICM_USER_BANK_3));

    reg_addr = ICM_I2C_SLV0_CTRL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, 0x00));
    reg_addr = ICM_I2C_SLV0_ADDR;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, 0x80 | IMU_MAG_ADDR));
    reg_addr = ICM_I2C_SLV0_REG;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, reg));
    reg_addr = ICM_I2C_SLV0_DO;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, 0xff));
    reg_addr = ICM_I2C_SLV0_CTRL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, 0xD0 | length));

    reg_addr = ICM_REG_BANK_SEL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, ICM_USER_BANK_0));

    nrf_delay_ms(1);

    reg_addr = ICM_ACCEL_XOUT_H; //ICM_EXT_SLV_SENS_DATA_00;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, TX_NO_STOP));
    char result[6] = { 0, 0, 0, 0, 0, 0 };
    MY_ERROR_CHECK(io_i2cRx(imu_addr[imu_number], result, 6));
    for (uint8_t i = 0; i < 6; i++)
    {
	NRF_LOG_DEBUG("%s(%d) result = 0x%x",
		      __FILENAME__, __LINE__,
		      result[i]);
    }
}

/**
 * @brief Setup continuous measurement
 * @param reg: first register to read
 * @param length: number of registers to read
 * Todo:      :returns: register values
 */
void readMagContinuous(uint8_t imu_number, char reg, char length)
{
    writeMagnReg(imu_number, ICM_AK_CNTL2, 0x08);

    char reg_addr = ICM_REG_BANK_SEL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, ICM_USER_BANK_3));

    reg_addr = ICM_I2C_SLV0_CTRL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, 0x00));
    reg_addr = ICM_I2C_SLV0_ADDR;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, 0x80 | IMU_MAG_ADDR));
    reg_addr = ICM_I2C_SLV0_REG;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, reg));
    reg_addr = ICM_I2C_SLV0_DO;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, 0xff));
    reg_addr = ICM_I2C_SLV0_CTRL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, 0xD0 | length));

    reg_addr = ICM_REG_BANK_SEL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, ICM_USER_BANK_0));
    
    nrf_delay_ms(1);
}

extern void icmInitiateAk09916(uint8_t imu_number)
{
    char reg_addr =  ICM_INT_PIN_CFG;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, 0x30));

    reg_addr = ICM_USER_CTRL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, TX_NO_STOP));
    char result[1] = {0};
    MY_ERROR_CHECK(io_i2cRx(imu_addr[imu_number], result, 1));
    NRF_LOG_DEBUG("%s(%d) result = 0x%x", __FILENAME__, __LINE__, *result);
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, result[1] |= 0x20));

    reg_addr = ICM_REG_BANK_SEL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, ICM_USER_BANK_3));

    reg_addr = ICM_I2C_MST_CTRL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, 0x1D));
    reg_addr = ICM_I2C_MST_DELAY_CTRL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, 0x01));

    readMagnReg(imu_number, IMU_MAG_ADDR, 3);

    writeMagnReg(imu_number, ICM_AK_CNTL3, 0x01);
    nrf_delay_us(100);
    nrf_delay_ms(10);
    readMagContinuous(imu_number, ICM_ACCEL_XOUT_H, 6);
    //compass_scale = 0.15;
}

/***
 * Low Byte of Temp sensor data.
 *
 * To convert the output of the temperature sensor to degrees C use the following
 * formula:
 * TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 21degC
 *
 * Operating Range Ambient: -40 to 85 °C
 * Sensitivity Untrimmed typical: 333.87 LSB/°C 1
 * Room Temp Offset at 21°C typical: 0 LSB
 */
extern void icmReadTempData(uint8_t imu_number)
{
  uint8_t  rawData[2];
  int16_t  TEMP_OUT = 0;
  int16_t  TEMP_degC = 0;
  float    RoomTemp_Offset = 0; //degrees Celsius
  float    Temp_Sensitivity = 333.87;
  float    deg21C = 21;
      
  // Read the two raw data registers sequentially into data array
  char reg = ICM_TEMP_OUT_H;
  io_i2cTx(imu_addr[imu_number], &reg, 2, TX_NO_STOP);
  io_i2cRx(imu_addr[imu_number], rawData, 2);
  NRF_LOG_DEBUG("rawData = 0x%02x%02x", rawData[0], rawData[1]);

  // Turn the MSB and LSB into a 16-bit value
  TEMP_OUT = (int16_t)((rawData[0] << 8) | rawData[1]);
  NRF_LOG_DEBUG("TEMP_OUT = 0x%04x%", TEMP_OUT);

  TEMP_degC = ((TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity) + deg21C;
  NRF_LOG_DEBUG("TEMP_degC = %d", TEMP_degC);
}

static int16_t convert( const uint8_t r1, const uint8_t r2 )
{
    int32_t i = r1;

    i = (i << 8) | r2;

    if (i & 0x8000)
	i |= ~0xffff;

    return (int16_t)(i);
}


extern void readAccelData(uint8_t imu_number)
{
    uint8_t rawData[6];

    /*
     * Read the six raw data registers into data array
     */
    char reg = ICM_ACCEL_XOUT_H;
    io_i2cTx(imu_addr[imu_number], &reg, 1, TX_NO_STOP);
    io_i2cRx(imu_addr[imu_number], rawData, 6);

    /*
     * Set device id and packet length.
     */
    imuRawData.device_id = (0x30 + imu_number);
    imuRawData.packet_len = 0xe; // Todo: set the length of the packet dynamically.

    /*
     * Turn the MSB and LSB into a signed 16-bit value
     */
    imuRawData.acc_x.u = (int16_t)convert( rawData[0], rawData[1]);
    imuRawData.acc_y.u = (int16_t)convert( rawData[2], rawData[3]);
    imuRawData.acc_z.u = (int16_t)convert( rawData[4], rawData[5]);
    NRF_LOG_DEBUG("Accelerometer(0x%x) len=%d int16 = %d, %d, %d\n",
		  imuRawData.device_id,
		  imuRawData.packet_len,
		  imuRawData.acc_x.u, imuRawData.acc_y.u, imuRawData.acc_z.u);
}

extern void readGyroData(uint8_t imu_number)
{
    uint8_t rawData[6];

    /*
     * Read the six raw data registers into data array
     */
    char reg = ICM_GYRO_XOUT_H;
    io_i2cTx(imu_addr[imu_number], &reg, 1, TX_NO_STOP);
    io_i2cRx(imu_addr[imu_number], rawData, 6);

    /*
     * Set device id and packet length.
     */
    imuRawData.device_id = (0x40 + imu_number);
    imuRawData.packet_len = 0xe; // Todo: set the length of the packet dynamically.

    /*
     * Turn the MSB and LSB into a signed 16-bit value
     */
    imuRawData.gyr_x.u = (uint16_t)(rawData[0] << 8) | rawData[1];
    imuRawData.gyr_y.u = (uint16_t)(rawData[2] << 8) | rawData[3];
    imuRawData.gyr_z.u = (uint16_t)(rawData[4] << 8) | rawData[5];
    NRF_LOG_DEBUG("Gyroscope(0x%x) len=%d int16 = %d, %d, %d\n",
		  imuRawData.device_id,
		  imuRawData.packet_len,
		  imuRawData.gyr_x.u, imuRawData.gyr_y.u, imuRawData.gyr_z.u);
}

extern void readMagnData(uint8_t imu_number)
{
    uint8_t rawData[8];
    char reg = 0;
    char data;

    char reg_addr = ICM_REG_BANK_SEL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, ICM_USER_BANK_3));

    reg_addr = ICM_I2C_SLV0_CTRL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, 0x00));
    reg_addr = ICM_I2C_SLV0_ADDR;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, IMU_MAG_ADDR));
    reg_addr = ICM_I2C_SLV0_REG;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, reg));
    reg_addr = ICM_I2C_SLV0_DO;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, data));
    reg_addr = ICM_I2C_SLV0_CTRL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, 0x81));
    char reg1 = ICM_AK_ST1;
    uint8_t statusData = 0;
    io_i2cTx(IMU_MAG_ADDR, &reg1, 1, TX_NO_STOP);
    if (io_i2cRx(IMU_MAG_ADDR, &statusData, 1) & 0x01)
    {
	char reg = ICM_AK_HXL;
	io_i2cTx(IMU_MAG_ADDR, &reg, 1, TX_NO_STOP);
	io_i2cRx(IMU_MAG_ADDR, rawData, 8);
	uint8_t c = rawData[7];	
	if (!(c & 0x08))
	{
	    /*
	     * Set device id and packet length.
	     */
	    imuRawData.device_id = (0x50 + imu_number);
	    imuRawData.packet_len = 0xe; // Todo: set the length of the packet dynamically.

	    /*
	     * Turn the MSB and LSB into a signed 16-bit value
	     */
	    imuRawData.mag_x.u = (int16_t)(rawData[1] << 8) | rawData[0];
	    imuRawData.mag_y.u = (int16_t)(rawData[3] << 8) | rawData[2];
	    imuRawData.mag_z.u = (int16_t)(rawData[5] << 8) | rawData[4];
            NRF_LOG_DEBUG("Magnetometer(0x%x) len=%d int16 = %d, %d, %d\n",
                          imuRawData.device_id,
                          imuRawData.packet_len,
                          imuRawData.mag_x.u, imuRawData.mag_y.u, imuRawData.mag_z.u);
	}
    }
    reg_addr = ICM_REG_BANK_SEL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, ICM_USER_BANK_0));
    
    nrf_delay_ms(1);
}

extern void readEulerData(uint8_t number)
{
    // Todo: Not implemented yet
}

void writeMagnReg(uint8_t imu_number, char reg, char data)
{
    char reg_addr = ICM_REG_BANK_SEL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, ICM_USER_BANK_3));

    reg_addr = ICM_I2C_SLV0_CTRL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, 0x00));
    reg_addr = ICM_I2C_SLV0_ADDR;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, IMU_MAG_ADDR));
    reg_addr = ICM_I2C_SLV0_REG;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, reg));
    reg_addr = ICM_I2C_SLV0_DO;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, data));
    reg_addr = ICM_I2C_SLV0_CTRL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, 0x81));

    reg_addr = ICM_REG_BANK_SEL;
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], &reg_addr, 1, ICM_USER_BANK_0));
}

/** @} */
