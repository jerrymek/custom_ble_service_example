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

void icmDeviceReset(void)
{
    char reg_addr = ICM_PWR_MGMT_1;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, ICM_PWR_MGMT_1_DEVICE_RESET));
    nrf_delay_ms(200);
}

extern void icmReadChipId(void)
{
    char reg_addr = ICM_WHO_AM_I;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, TX_NO_STOP));
    char result[1] = {0};
    MY_ERROR_CHECK(io_i2cRx(IMU11_ADDR, result, 1));
    NRF_LOG_DEBUG("%s(%d) result = 0x%x", __FILENAME__, __LINE__, *result);
}

void icmInitiateIcm20948(void)
{
    char data[2] = {ICM_PWR_MGMT_1, 0x01};
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, data, 2, TX_NO_STOP));
    data[0] = ICM_PWR_MGMT_2;
    data[1] = 0x00;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, data, 2, TX_NO_STOP));

    data[0] = ICM_REG_BANK_SEL;
    data[1] = ICM_USER_BANK_2;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, data, 2, TX_NO_STOP));

    data[0] = ICM_GYRO_CONFIG_1;
    data[1] = 0x19;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, data, 2, TX_NO_STOP));

    data[0] = ICM_TEMP_CONFIG;
    data[1] = 0x03;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, data, 2, TX_NO_STOP));

    data[0] = ICM_GYRO_SMPLRT_DIV;
    data[1] = 0x04;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, data, 2, TX_NO_STOP));

    data[0] = ICM_ACCEL_CONFIG;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, data, 1, TX_NO_STOP));
    uint8_t c = 0;
    MY_ERROR_CHECK(io_i2cRx(IMU11_ADDR, &c, 1));
    c = c & ~0x06;
    c = c | 0x00 << 1;
    c = c | 0x01;
    c = c | 0x18;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &c, 1, TX_NO_STOP));

    data[0] = ICM_ACCEL_SMPLRT_DIV_1;
    data[1] = 0;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, data, 2, TX_NO_STOP));
    data[0] = ICM_ACCEL_SMPLRT_DIV_2;
    data[1] = 5;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, data, 2, TX_NO_STOP));

    data[0] = ICM_ACCEL_CONFIG;
    data[1] = 1 | (1 << 1) | (3 << 3);
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, data, 1, TX_NO_STOP));
    uint16_t acc_scale = 4.0 / (1 << 15);
    uint16_t gyro_scale = 500.0 / (1 << 15);

    data[0] = ICM_REG_BANK_SEL;
    data[1] = ICM_USER_BANK_0;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, data, 2, TX_NO_STOP));

    data[0] = ICM_INT_PIN_CFG;
    data[1] = 0x22;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, data, 2, TX_NO_STOP));

    data[0] = ICM_INT_ENABLE_1;
    data[1] = 0x01;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, data, 2, TX_NO_STOP));
}

void readMagnReg (uint8_t reg, uint8_t length)
{
    char reg_addr = ICM_REG_BANK_SEL;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, ICM_USER_BANK_3));

    reg_addr = ICM_I2C_SLV0_CTRL;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, 0x00));
    reg_addr = ICM_I2C_SLV0_ADDR;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, 0x80 | IMU_MAG_ADDR));
    reg_addr = ICM_I2C_SLV0_REG;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, reg));
    reg_addr = ICM_I2C_SLV0_DO;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, 0xff));
    reg_addr = ICM_I2C_SLV0_CTRL;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, 0xD0 | length));

    reg_addr = ICM_REG_BANK_SEL;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, ICM_USER_BANK_0));

    nrf_delay_ms(1);

    reg_addr = ICM_ACCEL_XOUT_H; //ICM_EXT_SLV_SENS_DATA_00;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, TX_NO_STOP));
    char result[6] = { 0, 0, 0, 0, 0, 0 };
    MY_ERROR_CHECK(io_i2cRx(IMU11_ADDR, result, 6));
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
void readMagContinuous(char reg, char length)
{
    writeMagnReg(ICM_AK_CNTL2, 0x08);

    char reg_addr = ICM_REG_BANK_SEL;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, ICM_USER_BANK_3));

    reg_addr = ICM_I2C_SLV0_CTRL;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, 0x00));
    reg_addr = ICM_I2C_SLV0_ADDR;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, 0x80 | IMU_MAG_ADDR));
    reg_addr = ICM_I2C_SLV0_REG;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, reg));
    reg_addr = ICM_I2C_SLV0_DO;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, 0xff));
    reg_addr = ICM_I2C_SLV0_CTRL;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, 0xD0 | length));

    reg_addr = ICM_REG_BANK_SEL;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, ICM_USER_BANK_0));
    
    nrf_delay_ms(1);
}

void writeMagnReg(char reg, char data)
{
    char reg_addr = ICM_REG_BANK_SEL;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, ICM_USER_BANK_3));

    reg_addr = ICM_I2C_SLV0_CTRL;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, 0x00));
    reg_addr = ICM_I2C_SLV0_ADDR;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, IMU_MAG_ADDR));
    reg_addr = ICM_I2C_SLV0_REG;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, reg));
    reg_addr = ICM_I2C_SLV0_DO;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, data));
    reg_addr = ICM_I2C_SLV0_CTRL;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, 0x81));

    reg_addr = ICM_REG_BANK_SEL;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, ICM_USER_BANK_0));
}

extern void icmInitiateAk09916(void)
{
    char reg_addr =  ICM_INT_PIN_CFG;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, 0x30));

    reg_addr = ICM_USER_CTRL;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, TX_NO_STOP));
    char result[1] = {0};
    MY_ERROR_CHECK(io_i2cRx(IMU11_ADDR, result, 1));
    NRF_LOG_DEBUG("%s(%d) result = 0x%x", __FILENAME__, __LINE__, *result);
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, result[1] |= 0x20));

    reg_addr = ICM_REG_BANK_SEL;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, ICM_USER_BANK_3));

    reg_addr = ICM_I2C_MST_CTRL;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, 0x1D));
    reg_addr = ICM_I2C_MST_DELAY_CTRL;
    MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &reg_addr, 1, 0x01));

    readMagnReg(IMU_MAG_ADDR, 3);

    writeMagnReg(ICM_AK_CNTL3, 0x01);
    nrf_delay_us(100);
    nrf_delay_ms(10);
    readMagContinuous(ICM_ACCEL_XOUT_H, 6);
    //compass_scale = 0.15;
}

extern void icmReadAcclRawData(void)
{
    char result[6] = {0, 0, 0, 0, 0, 0};

    for (uint8_t i = ICM_ACCEL_XOUT_H; i <= ICM_ACCEL_ZOUT_L; i++)
    {
	MY_ERROR_CHECK(io_i2cTx(IMU11_ADDR, &i, 1, TX_NO_STOP));
	MY_ERROR_CHECK(io_i2cRx(IMU11_ADDR, &result[i], 1));
    };
    NRF_LOG_DEBUG("result = 0x%02x%02x%02x%02x%02x%02x\n",
		  result[0], result[1], result[2], result[3], result[4], result[5]);
}

extern void icmReadTempData(void)
{
  uint8_t rawData[2]; // x/y/z gyro register data stored here
  // Read the two raw data registers sequentially into data array
  char reg = ICM_TEMP_OUT_H;
  io_i2cTx(IMU11_ADDR, &reg, 2, TX_NO_STOP);
  io_i2cRx(IMU11_ADDR, rawData, 2);
  // Turn the MSB and LSB into a 16-bit value
  NRF_LOG_DEBUG("result = 0x%02x%02x", rawData[1], rawData[0]);
}

extern void readAccelData(void)
{
    int16_t destination[3] = { 0, 0, 0 };
    uint8_t rawData[6];  // x/y/z accel register data stored here
    // Read the six raw data registers into data array
    char reg = ICM_ACCEL_XOUT_H;
    io_i2cTx(IMU11_ADDR, &reg, 1, TX_NO_STOP);
    io_i2cRx(IMU11_ADDR, rawData, 6);

    // Turn the MSB and LSB into a signed 16-bit value
    destination[0] = (int16_t)(rawData[0] << 8) | rawData[1];
    destination[1] = (int16_t)(rawData[2] << 8) | rawData[3];
    destination[2] = (int16_t)(rawData[4] << 8) | rawData[5];
    NRF_LOG_DEBUG("result = %d, %d, %d",
		  destination[0], destination[1], destination[2]);
    
}

extern void readGyroData(void)
{
    int16_t destination[3] = { 0, 0, 0 };
    uint8_t rawData[6];  // x/y/z accel register data stored here
    // Read the six raw data registers into data array
    char reg = ICM_GYRO_XOUT_H;
    io_i2cTx(IMU11_ADDR, &reg, 1, TX_NO_STOP);
    io_i2cRx(IMU11_ADDR, rawData, 6);

    // Turn the MSB and LSB into a signed 16-bit value
    destination[0] = (int16_t)(rawData[0] << 8) | rawData[1];
    destination[1] = (int16_t)(rawData[2] << 8) | rawData[3];
    destination[2] = (int16_t)(rawData[4] << 8) | rawData[5];
    NRF_LOG_DEBUG("result = %d, %d, %d",
		  destination[0], destination[1], destination[2]);
    
}

/** @} */
