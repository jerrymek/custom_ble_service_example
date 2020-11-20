/**
 * Copyright  2020 PreCure ApS
 *
 * All Rights Reserved
 */
#ifndef _ICM20948_DRV_H_
#define _ICM20948_DRV_H_

#include "nrf_twi_mngr.h"

/*
 * USER BANK 0
 */
#define ICM_WHO_AM_I                0x00 // R WHO_AM_I[7:0]
#define ICM_WHO_AM_I_EXPECTED       0xEA // The expected register value for ICM20948
#define ICM_USER_CTRL               0x03 // R/W DMP_EN FIFO_EN I2C_MST_EN I2C_IF_DIS DMP_RST SRAM_RST I2C_MST_RST -
#define ICM_LP_CONFIG               0x05 // R/W I2C_MST_CYCLE ACCEL_CYCLE GYRO_CYCLE -
#define ICM_PWR_MGMT_1              0x06 // R/W DEVICE_RESET SLEEP LP_EN - TEMP_DIS CLKSEL[2:0]
#define ICM_PWR_MGMT_2              0x07 // R/W - DISABLE_ACCEL DISABLE_GYRO
#define ICM_INT_PIN_CFG             0x0F // R/W INT1_ACTL INT1_OPEN INT1_LATCH_INT_ENINT_ANYRD_2CLEAR ACTL_FSYNC FSYNC_INT_MODE_EN BYPASS_EN -
#define ICM_INT_ENABLE              0x10 // R/W REG_WOF_EN - WOM_INT_EN PLL_RDY_EN DMP_INT1_EN I2C_MST_INT_EN
#define ICM_INT_ENABLE_1            0x11 // R/W - RAW_DATA_0_RDY_EN
#define ICM_INT_ENABLE_2            0x12 // R/W - FIFO_OVERFLOW_EN[4:0]
#define ICM_INT_ENABLE_3            0x13 // R/W - FIFO_WM_EN[4:0]
#define ICM_I2C_MST_STATUS          0x17 // R/C PASS_THROUGH I2C_SLV4_DONE I2C_LOST_ARB I2C_SLV4_NACK I2C_SLV3_NACK I2C_SLV2_NACK I2C_SLV1_NACK I2C_SLV0_NACK
#define ICM_INT_STATUS              0x19 // R/C - WOM_INT PLL_RDY_INT DMP_INT1 I2C_MST_INT
#define ICM_INT_STATUS_1            0x1A // R/C - RAW_DATA_0_RDY_INT
#define ICM_INT_STATUS_2            0x1B // R/C - FIFO_OVERFLOW_INT[4:0]
#define ICM_INT_STATUS_3            0x1C // R/C - FIFO_WM_INT[4:0]
#define ICM_DELAY_TIMEH             0x28 // R DELAY_TIMEH[7:0]
#define ICM_DELAY_TIMEL             0x29 // R DELAY_TIMEL[7:0]
#define ICM_ACCEL_XOUT_H            0x2D // R ACCEL_XOUT_H[7:0]
#define ICM_ACCEL_XOUT_L            0x2E // R ACCEL_XOUT_L[7:0]
#define ICM_ACCEL_YOUT_H            0x2F // R ACCEL_YOUT_H[7:0]
#define ICM_ACCEL_YOUT_L            0x30 // R ACCEL_YOUT_L[7:0]
#define ICM_ACCEL_ZOUT_H            0x31 // R ACCEL_ZOUT_H[7:0]
#define ICM_ACCEL_ZOUT_L            0x32 // R ACCEL_ZOUT_L[7:0]
#define ICM_GYRO_XOUT_H             0x33 // R GYRO_XOUT_H[7:0]
#define ICM_GYRO_XOUT_L             0x34 // R GYRO_XOUT_L[7:0]
#define ICM_GYRO_YOUT_H             0x35 // R GYRO_YOUT_H[7:0]
#define ICM_GYRO_YOUT_L             0x36 // R GYRO_YOUT_L[7:0]
#define ICM_GYRO_ZOUT_H             0x37 // R GYRO_ZOUT_H[7:0]
#define ICM_GYRO_ZOUT_L             0x38 // R GYRO_ZOUT_L[7:0]
#define ICM_TEMP_OUT_H              0x39 // R TEMP_OUT_H[7:0]
#define ICM_TEMP_OUT_L              0x3A // R TEMP_OUT_L[7:0]
#define ICM_EXT_SLV_SENS_DATA_00    0x3B // R EXT_SLV_SENS_DATA_00[7:0]
#define ICM_EXT_SLV_SENS_DATA_01    0x3C // R EXT_SLV_SENS_DATA_01[7:0]
#define ICM_EXT_SLV_SENS_DATA_02    0x3D // R EXT_SLV_SENS_DATA_02[7:0]
#define ICM_EXT_SLV_SENS_DATA_03    0x3E // R EXT_SLV_SENS_DATA_03[7:0]
#define ICM_EXT_SLV_SENS_DATA_04    0x3F // R EXT_SLV_SENS_DATA_04[7:0]
#define ICM_EXT_SLV_SENS_DATA_05    0x40 // R EXT_SLV_SENS_DATA_05[7:0]
#define ICM_EXT_SLV_SENS_DATA_06    0x41 // R EXT_SLV_SENS_DATA_06[7:0]
#define ICM_EXT_SLV_SENS_DATA_07    0x42 // R EXT_SLV_SENS_DATA_07[7:0]
#define ICM_EXT_SLV_SENS_DATA_08    0x43 // R EXT_SLV_SENS_DATA_08[7:0]
#define ICM_EXT_SLV_SENS_DATA_09    0x44 // R EXT_SLV_SENS_DATA_09[7:0]
#define ICM_EXT_SLV_SENS_DATA_10    0x45 // R EXT_SLV_SENS_DATA_10[7:0]
#define ICM_EXT_SLV_SENS_DATA_11    0x46 // R EXT_SLV_SENS_DATA_11[7:0]
#define ICM_EXT_SLV_SENS_DATA_12    0x47 // R EXT_SLV_SENS_DATA_12[7:0]
#define ICM_EXT_SLV_SENS_DATA_13    0x48 // R EXT_SLV_SENS_DATA_13[7:0]
#define ICM_EXT_SLV_SENS_DATA_14    0x49 // R EXT_SLV_SENS_DATA_14[7:0]
#define ICM_EXT_SLV_SENS_DATA_15    0x4A // R EXT_SLV_SENS_DATA_15[7:0]
#define ICM_EXT_SLV_SENS_DATA_16    0x4B // R EXT_SLV_SENS_DATA_16[7:0]
#define ICM_EXT_SLV_SENS_DATA_17    0x4C // R EXT_SLV_SENS_DATA_17[7:0]
#define ICM_EXT_SLV_SENS_DATA_18    0x4D // R EXT_SLV_SENS_DATA_18[7:0]
#define ICM_EXT_SLV_SENS_DATA_19    0x4E // R EXT_SLV_SENS_DATA_19[7:0]
#define ICM_EXT_SLV_SENS_DATA_20    0x4F // R EXT_SLV_SENS_DATA_20[7:0]
#define ICM_EXT_SLV_SENS_DATA_21    0x50 // R EXT_SLV_SENS_DATA_21[7:0]
#define ICM_EXT_SLV_SENS_DATA_22    0x51 // R EXT_SLV_SENS_DATA_22[7:0]
#define ICM_EXT_SLV_SENS_DATA_23    0x52 // R EXT_SLV_SENS_DATA_23[7:0]
#define ICM_FIFO_EN_1               0x66 // R/W - SLV_3_FIFO_EN SLV_2_FIFO_EN SLV_1_FIFO_EN SLV_0_FIFO_EN
#define ICM_FIFO_EN_2               0x67 // R/W - ACCEL_FIFO_EN GYRO_Z_FIFO_EN GYRO_Y_FIFO_EN GYRO_X_FIFO_EN TEMP_FIFO_EN
#define ICM_FIFO_RST                0x68 // R/W - FIFO_RESET[4:0]
#define ICM_FIFO_MODE               0x69 // R/W - FIFO_MODE[4:0]
#define ICM_FIFO_COUNTH             0x70 // R - FIFO_CNT[12:8]
#define ICM_FIFO_COUNTL             0x71 // R FIFO_CNT[7:0]
#define ICM_FIFO_R_W                0x72 // R/W FIFO_R_W[7:0]
#define ICM_DATA_RDY_STATUS         0x74 // R/C WOF_STATU

/*
 * USER BANK 1
 */

#define ICM_SELF_TEST_X_GYRO        0x02 // R/W XG_ST_DATA[7:0]
#define ICM_SELF_TEST_Y_GYRO        0x03 // R/W YG_ST_DATA[7:0]
#define ICM_SELF_TEST_Z_GYRO        0x04 // R/W ZG_ST_DATA[7:0]
#define ICM_SELF_TEST_X_ACCEL       0x0E // R/W XA_ST_DATA[7:0]
#define ICM_SELF_TEST_Y_ACCEL       0x0F // R/W YA_ST_DATA[7:0]
#define ICM_SELF_TEST_Z_ACCEL       0x10 // R/W ZA_ST_DATA[7:0]
#define ICM_XA_OFFS_H               0x14 // R/W XA_OFFS[14:7]
#define ICM_XA_OFFS_L               0x15 // R/W XA_OFFS[6:0] -
#define ICM_YA_OFFS_H               0x17 // R/W YA_OFFS[14:7]
#define ICM_YA_OFFS_L               0x18 // R/W YA_OFFS[6:0] -
#define ICM_ZA_OFFS_H               0x1A // R/W ZA_OFFS[14:7]
#define ICM_ZA_OFFS_L               0x1B // R/W ZA_OFFS[6:0] -
#define ICM_TIMEBASE_CORRECTION_PLL 0x28 // R/W TBC_PLL[7:0]
#define ICM_REG_BANK_SEL            0x7F // R/W - USER_BANK[1:0] -

/*
 * USER BANK 2
 */

#define ICM_GYRO_SMPLRT_DIV         0x00  // R/W GYRO_SMPLRT_DIV[7:0]
#define ICM_GYRO_CONFIG_1           0x01  // R/W - GYRO_DLPFCFG[2:0] GYRO_FS_SEL[1:0] GYRO_FCHOICE
#define ICM_GYRO_CONFIG_2           0x02  // R/W - XGYRO_CTEN YGYRO_CTEN ZGYRO_CTEN GYRO_AVGCFG[2:0]
#define ICM_XG_OFFS_USRH            0x03  // R/W X_OFFS_USER[15:8]
#define ICM_XG_OFFS_USRL            0x04  // R/W X_OFFS_USER[7:0]
#define ICM_YG_OFFS_USRH            0x05  // R/W Y_OFFS_USER[15:8]
#define ICM_YG_OFFS_USRL            0x06  // R/W Y_OFFS_USER[7:0]
#define ICM_ZG_OFFS_USRH            0x07  // R/W Z_OFFS_USER[15:8]
#define ICM_ZG_OFFS_USRL            0x08  // R/W Z_OFFS_USER[7:0]
#define ICM_ODR_ALIGN_EN            0x09  // R/W - ODR_ALIGN_EN
#define ICM_ACCEL_SMPLRT_DIV_1      0x10  // R/W - ACCEL_SMPLRT_DIV[11:8]
#define ICM_ACCEL_SMPLRT_DIV_2      0x11  // R/W ACCEL_SMPLRT_DIV[7:0]
#define ICM_ACCEL_INTEL_CTRL        0x12  // R/W - ACCEL_INTEL_EN ACCEL_INTEL_MODE_INT
#define ICM_ACCEL_WOM_THR           0x13  // R/W WOM_THRESHOLD[7:0]
#define ICM_ACCEL_CONFIG            0x14  // R/W - ACCEL_DLPFCFG[2:0] ACCEL_FS_SEL[1:0] ACCEL_FCHOICE
#define ICM_ACCEL_CONFIG_2          0x15  // R/W - AX_ST_EN_R EGAY_ST_EN_REG AZ_ST_EN_REG DEC3_CFG[1:0]
#define ICM_FSYNC_CONFIG            0x52  // R/W DELAY_TIME_EN - WOF_DEGLITCH_EN WOF_EDGE_INT EXT_SYNC_SET[3:0]
#define ICM_TEMP_CONFIG             0x53  // R/W - TEMP_DLPFCFG[2:0]
#define ICM_MOD_CTRL_USR            0x54  // R/W - REG_LP_DMP_EN
#define ICM_REG_BANK_SEL            0x7F  // R/W - USER_BANK[1:0] -

/*
 * USER BANK 3
 */

#define ICM_I2C_MST_ODR_CONFIG      0x00  // R/W - I2C_MST_ODR_CONFIG[3:0]
#define ICM_I2C_MST_CTRL            0x01  // R/W MULT_MST_EN - I2C_MST_P_NSR I2C_MST_CLK[3:0]
#define ICM_I2C_MST_DELAY_CTRL      0x02  // R/W DELAY_ES_SHADOW - I2C_SLV4_DELAY_EN I2C_SLV3_DELAY_EN I2C_SLV2_DELAY_ENI2C_SLV1_DELAY_EN I2C_SLV0_DELAY_EN
#define ICM_I2C_SLV0_ADDR           0x03  // R/W I2C_SLV0_RNW I2C_ID_0[6:0]
#define ICM_I2C_SLV0_REG            0x04  // R/W I2C_SLV0_REG[7:0]
#define ICM_I2C_SLV0_CTRL           0x05  // R/W I2C_SLV0_EN I2C_SLV0_BYTE_SW I2C_SLV0_REG_DIS I2C_SLV0_GRP I2C_SLV0_LENG[3:0]
#define ICM_I2C_SLV0_DO             0x06  // R/W I2C_SLV0_DO[7:0]
#define ICM_I2C_SLV1_ADDR           0x07  // R/W I2C_SLV1_RNW I2C_ID_1[6:0]
#define ICM_I2C_SLV1_REG            0x08  // R/W I2C_SLV1_REG[7:0]
#define ICM_I2C_SLV1_CTRL           0x09   // R/W I2C_SLV1_EN I2C_SLV1_BYTE_SW I2C_SLV1_REG_DIS I2C_SLV1_GRP I2C_SLV1_LENG[3:0]
#define ICM_I2C_SLV1_DO             0x0A  // R/W I2C_SLV1_DO[7:0]
#define ICM_I2C_SLV2_ADDR           0x0B  // R/W I2C_SLV2_RNW I2C_ID_2[6:0]
#define ICM_I2C_SLV2_REG            0x0C  // R/W I2C_SLV2_REG[7:0]
#define ICM_I2C_SLV2_CTRL           0x0D  // R/W I2C_SLV2_EN I2C_SLV2_BYTE_SW I2C_SLV2_REG_DIS I2C_SLV2_GRP I2C_SLV2_LENG[3:0]
#define ICM_I2C_SLV2_DO             0x0E  // R/W I2C_SLV2_DO[7:0]
#define ICM_I2C_SLV3_ADDR           0x0F  // R/W I2C_SLV3_RNW I2C_ID_3[6:0]
#define ICM_I2C_SLV3_REG            0x10  // R/W I2C_SLV3_REG[7:0]
#define ICM_I2C_SLV3_CTRL           0x11  // R/W I2C_SLV3_EN I2C_SLV3_BYTE_SW I2C_SLV3_REG_DIS I2C_SLV3_GRP I2C_SLV3_LENG[3:0]
#define ICM_I2C_SLV3_DO             0x12  // R/W I2C_SLV3_DO[7:0]
#define ICM_I2C_SLV4_ADDR           0x13  // R/W I2C_SLV4_RNW I2C_ID_4[6:0]
#define ICM_I2C_SLV4_REG            0x14  // R/W I2C_SLV4_REG[7:0]
#define ICM_I2C_SLV4_CTRL           0x15  // R/W I2C_SLV4_EN I2C_SLV4_BYTE_SW I2C_SLV4_REG_DIS I2C_SLV4_DLY[4:0]
#define ICM_I2C_SLV4_DO             0x16  // R/W I2C_SLV4_DO[7:0]
#define ICM_I2C_SLV4_DI             0x17  // R I2C_SLV4_DI[7:0]
#define ICM_REG_BANK_SEL            0x7F  // R/W - USER_BANK[1:0] -

// 0x69 is the ICM20948's address in the mbed Application Shield, it contains
// R/W bit and "nrf_drv_twi" (and consequently "nrf_twi_mngr") requires slave
// address without this bit, hence shifting.
#define IMU11_ADDR       (0x69U) // Todo: >> 1) // Channel 1, device 1
#define IMU12_ADDR       (0x68U >> 1) // Channel 1, device 2
#define IMU21_ADDR       (0x69U >> 1) // Channel 2, device 1

#define TX_NO_STOP       1 // TX transfer will not end with a stop condition.
#define TX_MAY_STOP      0 // TX transfer may end with a stop condition.

/**
 * @brief Initialize both I2C channels
 * Used ports SDA and SCL must been configured first
 */
extern void io_i2cInit();

/**
 * @brief Set I2C channel
 */
extern void io_i2cSetChannel(uint8_t Channel);

/**
 * @brief I2C Transmit 
 *
 * @param Address = address byte to send
 * @param data = data to send
 * @param Len = length of data to send 
 * @param noStop = TX will not end with a stop condition
 */
extern void io_i2cTx(uint8_t Address, char *data, uint16_t Len, uint8_t noStop);

/**
 * @brief I2C Read
 *
 * @param Address = address byte from the device
 * @param dest = received data
 * @param Len = required length of the received data
 * @return = zero mean not all required data ready
 */
extern uint16_t io_i2cRx(uint8_t Address, char *dest, uint16_t Len);

#endif /* _ICM20948_DRV_H_ */
