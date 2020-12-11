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

uint8_t icmConfigureDevice(uint8_t imu_number, uint8_t addr, char reg, uint8_t data, bool readWrite, bool send_reg_addr);

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
    ret_code_t rc = 0;
    do
    {
	ret_code_t rc = nrf_drv_twi_tx(&m_twi, Address, data, Len, noStop);
    } while (rc == NRF_ERROR_BUSY);
    return rc;
}

nrfx_err_t io_i2cRx(uint8_t Address, char *dest, uint16_t Len)
{
    ret_code_t rc = 0;
    do
    {
	ret_code_t rc = nrf_drv_twi_rx(&m_twi, Address, dest, Len);
    } while (rc == NRF_ERROR_BUSY);
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
//    NRF_LOG_DEBUG("%s(%d) writeReg: 0x%x, 0x%x", __FILENAME__, __LINE__, reg, data);
}

void readReg (uint8_t imu_number, uint8_t reg, uint8_t *data)
{
    char tx_buf[1] = { reg };
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], tx_buf, 1, TX_NO_STOP));
    MY_ERROR_CHECK(io_i2cRx(imu_addr[imu_number], data, 1));
//    NRF_LOG_DEBUG("%s(%d) readReg: 0x%x, 0x%x", __FILENAME__, __LINE__, reg, *data);
}

void readRegs (uint8_t imu_number, uint8_t reg, uint8_t *data, uint8_t length)
{
    char tx_buf[1] = { reg };
    MY_ERROR_CHECK(io_i2cTx(imu_addr[imu_number], tx_buf, 1, TX_NO_STOP));
    MY_ERROR_CHECK(io_i2cRx(imu_addr[imu_number], data, length));
    /* for (uint8_t i = 0; i < length; i++) */
    /* { */
    /* 	NRF_LOG_DEBUG("%s(%d) readReg: 0x%x, 0x%x", __FILENAME__, __LINE__, reg, data[i]); */
    /* } */
}

void setBits(uint8_t imu_number, uint8_t reg, uint8_t bits)
{
    uint8_t d = 0;
    readReg(imu_number, reg, &d);
    d |= bits;
    writeReg (imu_number, reg, d);
}

void clearBits(uint8_t imu_number, uint8_t reg, uint8_t data)
{
    uint8_t d = 0;
    readReg(imu_number, reg, &d);
    d &= ~data;
    writeReg (imu_number, reg, d);
}

void icmReadChipId(uint8_t imu_number)
{
    uint8_t result = 0;
    readReg (imu_number, ICM_WHO_AM_I, &result);
    NRF_LOG_DEBUG("chip ID = 0x%x", result);
}

void icmDeviceReset(uint8_t imu_number)
{
    writeReg (imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0 );
    writeReg (imu_number, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_DEVICE_RESET);
    nrf_delay_ms(100);
    writeReg (imu_number, ICM_PWR_MGMT_1, ICM_PWR_MGMT_1_CLKSEL_0x1); // Auto clock select
    NRF_LOG_DEBUG("ICM 20948 reset done.");
}

void icmSleepMode(uint8_t imu_number, bool mode)
{
    writeReg (imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0 );
    uint8_t d = 0;
    readReg(imu_number, ICM_PWR_MGMT_1, &d);
    if (mode == true)
    {
	d |= ICM_PWR_MGMT_1_SLEEP;
    }
    else
    {
	d &= ~ICM_PWR_MGMT_1_SLEEP;
    }
    writeReg (imu_number, ICM_PWR_MGMT_1, d);
    NRF_LOG_DEBUG("ICM 20948 Sleep mode %d", mode);
}

void icmSetLowPowerMode(uint8_t imu_number, bool mode)
{
    writeReg (imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0 );
    uint8_t d = 0;
    readReg(imu_number, ICM_PWR_MGMT_1, &d);
    if (mode==true)
    {
	d |= ICM_PWR_MGMT_1_SLEEP;
    }
    else
    {
	d &= ~ICM_PWR_MGMT_1_SLEEP;
    }
    writeReg (imu_number, ICM_PWR_MGMT_1, d);
    NRF_LOG_DEBUG("ICM 20948 low power mode %d", mode);
}

void icmInitiateIcm20948(uint8_t imu_number)
{
    writeReg (imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0);
    writeReg (imu_number, ICM_LP_CONFIG, (ICM_LP_CONFIG_I2C_MST_CYCLE |
					  ICM_LP_CONFIG_ACCEL_CYCLE |
					  ICM_LP_CONFIG_GYRO_CYCLE));
    writeReg (imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_2);
    setBits(imu_number, ICM_ACCEL_CONFIG_1, (ICM_ACCEL_CONFIG_1_FS_SEL_1 | // 0b0110 = ±16g
					     ICM_ACCEL_CONFIG_1_FS_SEL_0));
    setBits(imu_number, ICM_GYRO_CONFIG_1, (ICM_GYRO_CONFIG_1_FS_SEL_2 | // 0b0110 = ±2000 dps
					    ICM_GYRO_CONFIG_1_FS_SEL_1));
    setBits(imu_number, ICM_ACCEL_CONFIG_1, (ICM_ACCEL_CONFIG_1_DLPFCFG_2 | // Accelerometer low pass filter configuration
					     ICM_ACCEL_CONFIG_1_DLPFCFG_1 |
					     ICM_ACCEL_CONFIG_1_DLPFCFG_0));
    setBits(imu_number, ICM_GYRO_CONFIG_1, (ICM_GYRO_CONFIG_1_DLPFCFG_5 | // Gyroscope low pass filter configuration
					    ICM_GYRO_CONFIG_1_DLPFCFG_4 |
					    ICM_GYRO_CONFIG_1_DLPFCFG_3));
    clearBits(imu_number, ICM_ACCEL_CONFIG_1, ICM_ACCEL_CONFIG_1_FCHOICE); // Disable LP-filter on accel
    clearBits(imu_number, ICM_GYRO_CONFIG_1, ICM_GYRO_CONFIG_1_FCHOICE);    // Disable LP-filter on gyro
    NRF_LOG_DEBUG("ICM 20948 Initiated");
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

void icmInitiateAk09916(uint8_t imu_number)
{
    writeReg (imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_3);
    clearBits(imu_number, ICM_INT_PIN_CFG, ICM_INT_PIN_CFG_BYPASS_EN); // Disable pass through
    setBits(imu_number, ICM_I2C_MST_CTRL, 0x17); // I2C_MST_P_NSR bit set and 0x7 => 345.60 kHz, should work up to 400 kHz.

    writeReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0);
    setBits(imu_number, ICM_USER_CTRL, ICM_USER_CTRL_I2C_MST_EN);

    uint8_t MAG_AK09916_WHO_AM_I[2] = { 0x48, 0x09 };
    uint8_t mag_id[2] = { 0, 0 };
    uint8_t i = 0;
    for (i = 0; i < 5; i++)
    {
//	mag_id[ICM_AK_WIA1] = icmConfigureDevice(imu_number, IMU_MAG_ADDR, ICM_AK_WIA1, 0, true, true);
//	mag_id[ICM_AK_WIA2] = icmConfigureDevice(imu_number, IMU_MAG_ADDR, ICM_AK_WIA2, 0, true, true);

	readMagnetometerRegister(imu_number, ICM_AK_WIA1, &mag_id[ICM_AK_WIA1]);
	readMagnetometerRegister(imu_number, ICM_AK_WIA2, &mag_id[ICM_AK_WIA2]);
	if ((mag_id[ICM_AK_WIA1] == MAG_AK09916_WHO_AM_I[0]) &&
	    (mag_id[ICM_AK_WIA2] == MAG_AK09916_WHO_AM_I[1]))
	{
	    NRF_LOG_DEBUG("%s(%d) Found chip for the magneotmeter: 0x%x, 0x%x", __FILENAME__, __LINE__, mag_id[ICM_AK_WIA1], mag_id[ICM_AK_WIA2]);
	    break;
	}
	writeReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0);
	setBits(imu_number, ICM_USER_CTRL, ICM_USER_CTRL_I2C_MST_RST);
    }
    /* if (i = 5) */
    /* { */
    /* 	NRF_LOG_DEBUG("%s(%d) Found chip for the magneotmeter: 0x%x, 0x%x", __FILENAME__, __LINE__, mag_id[ICM_AK_WIA1], mag_id[ICM_AK_WIA2]); */
    /* 	MY_ERROR_CHECK(GENERAL_FAILURE); */
    /* } */

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
    NRF_LOG_DEBUG("AK09916 initiated!");
}

void readMagnetometerRegister(uint8_t imu_number,  uint8_t reg,  uint8_t *d)
{
    writeMagnReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_3);
    writeMagnReg(imu_number, ICM_I2C_SLV4_ADDR, (ICM_I2C_SLV4_ADDR_RNW |
						 IMU_MAG_ADDR));
    writeMagnReg(imu_number, ICM_I2C_SLV4_REG, reg);

    uint8_t data = 0;
    data |= ICM_I2C_SLV4_CTRL_EN;  // EN bit [7] (set)
    data &= ~(ICM_I2C_SLV4_CTRL_INT_EN); // INT_EN bit [6] (cleared)
    data &= ~(ICM_I2C_SLV4_CTRL_DLY_3 |
     	      ICM_I2C_SLV4_CTRL_DLY_2 |
     	      ICM_I2C_SLV4_CTRL_DLY_1 |
     	      ICM_I2C_SLV4_CTRL_DLY_0); // DLY bits [4:0] (cleared = 0)
    data &= ~(ICM_I2C_SLV4_CTRL_REG_DIS); // REG_DIS bit [5] (cleared);
    writeMagnReg(imu_number, ICM_I2C_SLV4_CTRL, data);

    uint16_t max_cycles = 1000;
    uint16_t count = 0;
    bool slave4Done = false;
    bool txn_failed = false;

    writeMagnReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0);
    while (slave4Done == false)
    {
	readReg(imu_number, ICM_I2C_MST_STATUS, d);
	if((*d & ICM_I2C_MST_STATUS_I2C_SLV4_DONE)==ICM_I2C_MST_STATUS_I2C_SLV4_DONE) // Check I2C_SLAVE_DONE bit [6]
	{
	    slave4Done = true;
	}
	if (count > max_cycles)
	{
	    slave4Done = true;
	}
	count++;
    }
    
    if ((*d & (1<<4)) == (1<<4)) // Check I2C_SLV4_NACK bit [4]
    {
	txn_failed = true;
    }

    if (count > max_cycles)
    {
	txn_failed = true;
    }

    if (txn_failed)
    {
	return; // Todo: False;
    }
    /* if Rw: */
    /* 	self.setBank(3) */
    /* 	return self._i2c.readByte(self.address, self.AGB3_REG_I2C_SLV4_DI) */
		
    return; // Todo: true // if we get here, then it was a successful write

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
    uint8_t result[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    readRegs(imu_number, ICM_EXT_SLV_SENS_DATA_00, result, 8); // ICM_EXT_SLV_SENS_DATA_00 contains a status word.
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
void icmReadTempData(uint8_t imu_number)
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
    imu_data->data_x.u = ((int16_t)convert(rawData[0], rawData[1]));
    imu_data->data_y.u = ((int16_t)convert(rawData[2], rawData[3]));
    imu_data->data_z.u = ((int16_t)convert(rawData[4], rawData[5]));

    NRF_LOG_DEBUG("Sensor(0x%x) len=%d int16 = %d, %d, %d",
    		  imu_data->device_id,
    		  imu_data->packet_length,
    		  imu_data->data_x.u, imu_data->data_y.u, imu_data->data_z.u);
}

void readMagnSensor(ble_ss_t *p_sensor_service, char reg, uint8_t sensor_type, uint8_t imu_number, icm_imu_data_t *imu_data)
{
    uint8_t device_id = (sensor_type + imu_number);
    uint8_t statusData = 0;
    uint8_t rawData[8];

    do {
	readRegs(imu_number, ICM_AK_ST1, &statusData, 1);
    } while ((statusData & ICM_AK_ST1_DRDY) == ICM_AK_ST1_DRDY);

    readRegs(imu_number, reg, rawData, 8);
    uint8_t c = rawData[7]; // n.b. Little Endian!

    if (!(c & 0x08))
    {
	/*
	 * Set device id and packet length.
	 */
	imu_data->device_id = device_id;
	imu_data->packet_length = IMU_DATA_LENGTH;

	/*
	 * Turn the MSB and LSB into a signed 16-bit value
	 * N.B. Little Endian!
	 * Also skip data 0 (status 1) and 7 (status 2).
	 */
	imu_data->data_x.u = ((int16_t)convert(rawData[2], rawData[1]));
	imu_data->data_y.u = ((int16_t)convert(rawData[4], rawData[3]));
	imu_data->data_z.u = ((int16_t)convert(rawData[6], rawData[5]));
	NRF_LOG_DEBUG("Sensor(0x%x) len=%d int16 = %d, %d, %d",
		      imu_data->device_id,
		      imu_data->packet_length,
		      imu_data->data_x.u, imu_data->data_y.u, imu_data->data_z.u);
    }
    else
    {
	NRF_LOG_DEBUG("Magnetometer: something else is wrong with the data!!!");
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
void readSensorData(ble_ss_t *p_sensor_service, char reg, uint8_t sensor_type, uint8_t imu_number, icm_imu_data_t *imu_data)
{
#ifndef IMU_NOT_PRESENT
    ret_code_t err_code = GENERAL_FAILURE;
    if (sensor_type == IMU_MAGNETOMETER)
    {
	writeMagnReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_3);
	writeMagnReg(imu_number, ICM_I2C_SLV0_ADDR, (IMU_MAG_ADDR | ICM_I2C_SLV0_ADDR_READ_FLAG));
	writeMagnReg(imu_number, ICM_I2C_SLV0_REG, (ICM_I2C_SLV0_REG, IMU_MAG_ADDR));
	writeMagnReg(imu_number, ICM_I2C_SLV0_CTRL, (ICM_I2C_SLV0_CTRL_EN | ICM_I2C_SLV0_ADDR_READ_FLAG));
    	readMagnSensor(p_sensor_service, reg, sensor_type, imu_number, imu_data);
	writeMagnReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0);
    }
    else
    {
	getSensorData(p_sensor_service, reg, sensor_type, imu_number, imu_data);
    }
#else
#endif
    err_code = icm_stream_update(p_sensor_service,
			         imu_data);
}

void readEulerData(uint8_t number)
{
    // Todo: Not implemented yet
}

void writeMagnReg(uint8_t imu_number, char reg, uint8_t data)
{
    writeReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_3);
    writeReg(imu_number, ICM_I2C_SLV0_CTRL, 0x00);
    writeReg(imu_number, ICM_I2C_SLV0_ADDR, IMU_MAG_ADDR);
    writeReg(imu_number, ICM_I2C_SLV0_REG, reg);
    writeReg(imu_number, ICM_I2C_SLV0_DO, data);
    writeReg(imu_number, ICM_I2C_SLV0_CTRL, 0x81);
    writeReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0);
}

uint8_t icmConfigureDevice(uint8_t imu_number, uint8_t addr, char reg, uint8_t data, bool readWrite, bool send_reg_addr)
{
    if (readWrite = true)
    {
	addr |= 0x80;
    }

    writeReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_3);
    writeReg(imu_number, ICM_I2C_SLV4_ADDR,  addr);
    writeReg(imu_number, ICM_I2C_SLV4_REG,  reg);

    uint8_t ctrl_register_slv4 = 0x00;
    ctrl_register_slv4 |= (1<<7); // EN bit [7] (set)
    ctrl_register_slv4 &= ~(1<<6); // INT_EN bit [6] (cleared)
    ctrl_register_slv4 &= ~(0x0F); // DLY bits [4:0] (cleared = 0)

    if(send_reg_addr)
    {
	ctrl_register_slv4 &= ~(1<<5); // REG_DIS bit [5] (cleared)
    }
    else
    {
	ctrl_register_slv4 |= (1<<5); // REG_DIS bit [5] (set)
    }

    bool txn_failed = false;

    if (readWrite == false)
    {
	writeReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_3);
	writeReg(imu_number, ICM_I2C_SLV4_DO,  data);

	// Kick off txn
	writeReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_3);
	writeReg(imu_number, ICM_I2C_SLV4_CTRL, ctrl_register_slv4);

	uint16_t max_cycles = 1000;
	uint16_t count = 0;
	uint8_t i2c_mst_status;
	bool slave4Done = false;
	while (slave4Done == false)
	{
	    writeReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_0);

	    readRegs(imu_number, ICM_I2C_MST_STATUS, &i2c_mst_status, 8);
	    if (i2c_mst_status & (1<<6)) // Check I2C_SLAVE_DONE bit [6]
	    {
		slave4Done = true;
	    }

	    if (count > max_cycles)
	    {
		slave4Done = true;
	    }

	    count += 1;
	}

	if (i2c_mst_status & (1<<4)) // Check I2C_SLV4_NACK bit [4]
	{
	    txn_failed = true;
	}

	if (count > max_cycles)
	{
	    txn_failed = true;
	}

	if (txn_failed)
	{
	    return false;
	}

	if (readWrite = true)
	{
	    uint8_t data;
	    writeReg(imu_number, ICM_REG_BANK_SEL, ICM_USER_BANK_3);
	    readRegs(imu_number, ICM_I2C_SLV4_DI, &data, 1);

	    return data;
	}

	return true; // if we get here, then it was a successful write
    }
}
/** @} */
