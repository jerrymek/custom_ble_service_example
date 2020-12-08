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
#include "sensor_service.h"
#include "icm20948_drv.h"

/* TWI instance ID. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(1);

static uint8_t CurrentChannel;

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
		      I2C_SCL1_PIN,
		      I2C_SDA1_PIN);
}

nrfx_err_t io_i2cTx(uint8_t Address, const char *data, uint16_t Len, uint8_t noStop)
{
    ret_code_t rc = nrf_drv_twi_tx(&m_twi, Address, data, Len, noStop);
    return rc;
}

nrfx_err_t io_i2cRx(uint8_t Address, char *dest, uint16_t Len)
{
    ret_code_t rc = nrf_drv_twi_rx(&m_twi, Address, dest, Len);
    return rc;
}

void icmInitI2c(void)
{
    MY_ERROR_CHECK(io_i2cInit()); 
    io_i2cSetChannel(0);
}

void writeReg (uint8_t imu_number, uint8_t reg, uint8_t data)
{
    char tx_buf[2] = { reg, data };
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], tx_buf, 2, TX_NO_STOP));
    NRF_LOG_DEBUG("%s(%d) writeReg: 0x%x, 0x%x", __FILENAME__, __LINE__, reg, data);
}

void readReg (uint8_t imu_number, uint8_t reg, uint8_t *data)
{
    char tx_buf[1] = { reg };
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], tx_buf, 1, TX_NO_STOP));
    MY_ERROR_CHECK(io_i2cRx(imu_addr[imu_number], data, 1));
    NRF_LOG_DEBUG("%s(%d) readReg: 0x%x, 0x%x", __FILENAME__, __LINE__, reg, *data);
}

void readRegs (uint8_t imu_number, uint8_t reg, uint8_t *data, uint8_t length)
{
    char tx_buf[1] = { reg };
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], tx_buf, 1, TX_NO_STOP));
    MY_ERROR_CHECK(io_i2cRx(imu_addr[imu_number], data, length));
    for (uint8_t i = 0; i < length; i++)
    {
	NRF_LOG_DEBUG("%s(%d) readReg: 0x%x, 0x%x", __FILENAME__, __LINE__, reg, data[i]);
    }
}

void icmDeviceReset(uint8_t imu_number)
{
    writeReg (imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0 );
    writeReg (imu_number, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_DEVICE_RESET);
    nrf_delay_ms(100);
    writeReg (imu_number, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_CLKSEL_0x1); // Auto clock select
    NRF_LOG_DEBUG("ICM 20948 reset done.");
}

extern void icmReadChipId(uint8_t imu_number)
{
    uint8_t result = 0;
    readReg (imu_number, ICM_WHO_AM_I, &result);
}

void icmInitiateIcm20948(uint8_t imu_number)
{
    writeReg (imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0);
    writeReg (imu_number, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_CLKSEL_0x1);
    writeReg (imu_number, ICM_PWR_MGMT_2, ICM_PWR_MGMT_2_DISABLE_ALL);
    writeReg (imu_number, ICM_LP_CONFIG, (ICM_LP_CONFIG_I2C_MST_CYCLE |
					  ICM_LP_CONFIG_ACCEL_CYCLE |
					  ICM_LP_CONFIG_GYRO_CYCLE));
    writeReg (imu_number, ICM_LP_CONFIG, ICM_LP_CONFIG_I2C_MST_CYCLE); // Todo: tx_buf = BIT_I2C_MST_CYCLE|BIT_ACCEL_CYCLE|BIT_GYRO_CYCLE;
    writeReg (imu_number, ICM_USER_CTRL, 0x00); // Disable Digital Motion Processor

    // Todo: Load binary into Digital Motion Processor

    writeReg (imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_2);
    writeReg (imu_number, ICM_GYRO_CONFIG_1, 0x19);
    writeReg (imu_number, ICM_TEMP_CONFIG, 0x03);
    writeReg (imu_number, ICM_GYRO_SMPLRT_DIV, 0x04);
    uint8_t c = 0;
    readReg (imu_number, ICM_ACCEL_CONFIG, &c);
    c = c & ~0x06;
    c = c | 0x00 << 1;
    c = c | 0x01;
    c = c | 0x18;
    writeReg (imu_number, ICM_ACCEL_CONFIG, c);
    writeReg (imu_number, ICM_ACCEL_SMPLRT_DIV_2, 4);
    writeReg (imu_number, ICM_ACCEL_CONFIG, (1 | (1 << 1) | (3 << 3)));

    // Todo: uint16_t acc_scale = 4.0 / (1 << 15);
    // Todo: uint16_t gyro_scale = 500.0 / (1 << 15);

    writeReg (imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0);
    writeReg (imu_number, ICM_INT_PIN_CFG, 0x22);
    writeReg (imu_number, ICM_INT_ENABLE_1, 0x01);
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

    writeReg (imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_3);
    writeReg (imu_number, ICM_I2C_SLV0_CTRL, 0x0);
    writeReg (imu_number, ICM_I2C_SLV0_ADDR, (0x80 | IMU_MAG_ADDR));
    writeReg (imu_number, ICM_I2C_SLV0_REG, reg);
    writeReg (imu_number, ICM_I2C_SLV0_DO, 0xff);
    writeReg (imu_number, ICM_I2C_SLV0_CTRL, (0xD0 | length));
    writeReg (imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0);
}

extern void icmInitiateAk09916(uint8_t imu_number)
{
    writeReg (imu_number, ICM_INT_PIN_CFG, 0x02);

    uint8_t result = 0;
    readReg (imu_number, ICM_USER_CTRL, &result);
    writeReg (imu_number, ICM_USER_CTRL, (result |= 0x20));

    writeReg (imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_3);
    writeReg (imu_number, ICM_I2C_MST_CTRL, 0x1D);
    writeReg (imu_number, ICM_I2C_MST_DELAY_CTRL, 0x01);

    readMagnReg(imu_number, IMU_MAG_ADDR, 6);

    writeMagnReg(imu_number, ICM_AK_CNTL3, 0x01);
    nrf_delay_us(100);
    writeMagnReg(imu_number, ICM_AK_CNTL2, 0x80); // 100 Hz
    nrf_delay_ms(10);
    readMagContinuous(imu_number, ICM_ACCEL_XOUT_H, 6);
    //compass_scale = 0.15;
}

void readMagnReg (uint8_t imu_number, uint8_t reg, uint8_t length)
{
    writeMagnReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_3);
    writeMagnReg(imu_number, ICM_I2C_SLV0_CTRL, 0x00);
    writeMagnReg(imu_number, ICM_I2C_SLV0_ADDR, (0x80 | IMU_MAG_ADDR));
    writeMagnReg(imu_number, ICM_I2C_SLV0_REG, reg);
    writeMagnReg(imu_number, ICM_I2C_SLV0_DO, 0xff);
    writeMagnReg(imu_number, ICM_I2C_SLV0_CTRL, (0xD0 | length));

    writeMagnReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0);
    writeMagnReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0);

    uint8_t result[6] = { 0, 0, 0, 0, 0, 0 };
    readRegs(imu_number, ICM_EXT_SLV_SENS_DATA_01, result, 6); // ICM_EXT_SLV_SENS_DATA_00 contains a status word.
                                                               // N.B. Magnetometer data is in little endian format.
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
  int16_t  TEMP_OUT = 0;
  int16_t  TEMP_degC = 0;
  float    RoomTemp_Offset = 0; //degrees Celsius
  float    Temp_Sensitivity = 333.87;
  float    deg21C = 21;
      
  // Read the two raw data registers sequentially into data array
  uint8_t  rawData[2];
  readRegs(imu_number, ICM_TEMP_OUT_H, rawData, 2);

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

void getSensorData(ble_ss_t *p_sensor_service, char reg, uint8_t sensor_type, uint8_t imu_number, icm_imu_data_t *imu_data)
{
    /*
     * Read the six raw data registers into data array
     */
    uint8_t rawData[6];
    readRegs(imu_number, reg, rawData, 6);

    /*
     * Get device ID and packet length.
     */
    imu_data->device_id = (sensor_type | imu_number);
    imu_data->packet_length = IMU_DATA_LENGTH;
    
    /*
     * Get data from the sensor and
     * turn the MSB and LSB into a signed 16-bit value
     */
    imu_data->data_x.u = ((int16_t)rawData[0], rawData[1]);
    imu_data->data_y.u = ((int16_t)rawData[2], rawData[3]);
    imu_data->data_z.u = ((int16_t)rawData[4], rawData[5]);

    NRF_LOG_DEBUG("Sensor(0x%x) len=%d int16 = %d, %d, %d",
    		  imu_data->device_id,
    		  imu_data->packet_length,
    		  imu_data->data_x.u, imu_data->data_y.u, imu_data->data_z.u);
}

extern void readMagnSensor(ble_ss_t *p_sensor_service, char reg, uint8_t sensor_type, uint8_t imu_number, icm_imu_data_t *imu_data)
{
    uint8_t device_id = (sensor_type + imu_number);
    uint8_t statusData = 0;
    uint8_t rawData[7];

    do {
	readRegs(imu_number, ICM_AK_ST1, &statusData, 1);
    } while ((statusData & ICM_AK_ST1_DRDY) == ICM_AK_ST1_DRDY);

    readRegs(imu_number, reg, rawData, 7);
    uint8_t c = rawData[6]; // n.b. Little Endian!

    if (!(c & 0x08))
    {
	/* NRF_LOG_DEBUG("Magnetometer raw data 0x%x, 0x%x, 0x%x, 0x%x, 0x%x", */
	/* 	      rawData[0], rawData[1], rawData[2], */
	/* 	      rawData[3], rawData[4], rawData[5]); */

	/*
	 * Set device id and packet length.
	 */
	imu_data->device_id = device_id;
	imu_data->packet_length = IMU_DATA_LENGTH;

	/*
	 * Turn the MSB and LSB into a signed 16-bit value
	 */
	imu_data->data_x.u = ((int16_t)rawData[0], rawData[1]);
	imu_data->data_y.u = ((int16_t)rawData[2], rawData[3]);
	imu_data->data_z.u = ((int16_t)rawData[4], rawData[5]);
	/* imu_data->data_x.f = ((int16_t)convert(rawData[1], rawData[0])) * 0x1p-8f; */
	/* imu_data->data_y.f = ((int16_t)convert(rawData[3], rawData[2])) * 0x1p-8f; */
	/* imu_data->data_z.f = ((int16_t)convert(rawData[5], rawData[4])) * 0x1p-8f; */
	NRF_LOG_DEBUG("Sensor(0x%x) len=%d int16 = %d, %d, %d",
		      imu_data->device_id,
		      imu_data->packet_length,
		      imu_data->data_x.u, imu_data->data_y.u, imu_data->data_z.u);
    }
    else
    {
	NRF_LOG_DEBUG("Meagnetometer: something else is wrong with the data!!!");
    }
}

/*
 * @brief Read sensor data from the IMU.
 *
 * @param reg         - Register to read from, supported registers:
 *                      ICM_ACCEL_XOUT_H, ICM_GYRO_XOUT_H or ICM_AK_HXL
 *        sensor_type - Which type of sensor data to read, supported types:
 *                      IMU_ACCELEROMETER, IMU_GYROSCOPE or IMU_MAGNETOMETER
 *        imu_number  - Which IMU to read data from, supported ID's:
 *                      IMU1, IMU2 or IMU3
 */
extern void readSensorData(ble_ss_t *p_sensor_service, char reg, uint8_t sensor_type, uint8_t imu_number, icm_imu_data_t *imu_data)
{
    ret_code_t err_code = GENERAL_FAILURE;
    if (sensor_type == IMU_MAGNETOMETER)
    {
	writeMagnReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_3);
	writeMagnReg(imu_number, ICM_I2C_SLV0_ADDR, (IMU_MAG_ADDR | ICM_I2C_SLV0_ADDR_READ_FLAG));
	writeMagnReg(imu_number, ICM_I2C_SLV0_REG, (ICM_I2C_SLV0_REG, ICM_AK_HXL));
	writeMagnReg(imu_number, ICM_I2C_SLV0_CTRL, (ICM_I2C_SLV0_CTRL_EN | 8));
    	readMagnSensor(p_sensor_service, reg, sensor_type, imu_number, imu_data);
	writeMagnReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0);
    }
    else
    {
	getSensorData(p_sensor_service, reg, sensor_type, imu_number, imu_data);
    }
    err_code = icm_stream_update(p_sensor_service,
			         imu_data);
}

extern void readEulerData(uint8_t number)
{
    // Todo: Not implemented yet
}

void writeMagnReg(uint8_t imu_number, char reg, char data)
{
    writeMagnReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_3);
    writeMagnReg(imu_number, ICM_I2C_SLV0_CTRL, 0x00);
    writeMagnReg(imu_number, ICM_I2C_SLV0_ADDR, IMU_MAG_ADDR);
    writeMagnReg(imu_number, ICM_I2C_SLV0_REG, reg);
    writeMagnReg(imu_number, ICM_I2C_SLV0_DO, data);
    writeMagnReg(imu_number, ICM_I2C_SLV0_CTRL, 0x81);
    writeMagnReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0);
}

/** @} */
