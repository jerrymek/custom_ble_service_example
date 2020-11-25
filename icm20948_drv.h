/**
 * Copyright  2020 PreCure ApS
 *
 * All Rights Reserved
 */
#ifndef _ICM20948_DRV_H_
#define _ICM20948_DRV_H_

/*
 * USER BANK 0
 */
#define ICM_USER_BANK_0             0x00
#define ICM_WHO_AM_I                0x00 // R WHO_AM_I[7:0]
#define ICM_WHO_AM_I_EXPECTED       0xEA // The expected register value for ICM20948

#define ICM_USER_CTRL 0x03 // R/W DMP_EN FIFO_EN I2C_MST_EN I2C_IF_DIS DMP_RST SRAM_RST I2C_MST_RST -
#define ICM_USER_CTRL_DMP_EN        0x80 // Enables DMP
#define ICM_USER_CTRL_FIFO_EN       0x40 // Enable FIFO
#define ICM_USER_CTRL_I2C_MST_EN    0x20 // Enable the I2C Master
#define ICM_USER_CTRL_I2C_IF_DIS    0x10 // Disable I2C interface
#define ICM_USER_CTRL_DMP_RST       0x08 // Reset DMP
#define ICM_USER_CTRL_SRAM_RST      0x04 // Reset SRAM
#define ICM_USER_CTRL_I2C_MST_RST   0x02 // Reset I2C Master
#define ICM_USER_CTRL_RESERVED_0x01 0x01 // Reserved 0x01

#define ICM_LP_CONFIG  0x05 // R/W I2C_MST_CYCLE ACCEL_CYCLE GYRO_CYCLE -
#define ICM_LP_CONFIG_RESERVED_0x80 0x80
#define ICM_LP_CONFIG_I2C_MST_CYCLE 0x40
#define ICM_LP_CONFIG_ACCEL_CYCLE   0x20
#define ICM_LP_CONFIG_GYRO_CYCLE    0x10
#define ICM_LP_CONFIG_RESERVED_0x0f 0x0f

#define ICM_PWR_MGMT_1 0x06 // R/W DEVICE_RESET SLEEP LP_EN - TEMP_DIS CLKSEL[2:0]
#define ICM_PWR_MGMT_1_DEVICE_RESET 0x80
#define ICM_PWR_MGMT_1_SLEEP        0x40
#define ICM_PWR_MGMT_1_LP_EN        0x20
#define ICM_PWR_MGMT_1_RESERVD_0x08 0x10
#define ICM_PWR_MGMT_1_TEMP_DIS     0x08
#define ICM_PWR_MGMT_1_CLKSEL_0x4   0x04
#define ICM_PWR_MGMT_1_CLKSEL_0x2   0x02
#define ICM_PWR_MGMT_1_CLKSEL_0x1   0x01

#define ICM_PWR_MGMT_2 0x07 // R/W - DISABLE_ACCL DISABLE_GYRO DISABLE_BOTH
#define ICM_PWR_MGMT_2_RESERVD_0xC0 0xC0 // Reserved bits               0b1100 000
#define ICM_PWR_MGMT_2_DISABLE_ACCL 0x38 // Dis accel, en gyro          0b00111000
#define ICM_PWR_MGMT_2_DISABLE_GYRO 0x07 // En accel, dis gyro          0x00000111
#define ICM_PWR_MGMT_2_DISABLE_BOTH 0x1f // Disable both accel and gyro 0x00111111

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

#define ICM_REG_BANK_SEL            0x7F  // R/W - USER_BANK[1:0] -
#define ICM_REG_BANK_SEL_RESERVED_0 0x01
#define ICM_REG_BANK_SEL_RESERVED_1 0x02
#define ICM_REG_BANK_SEL_RESERVED_2 0x04
#define ICM_REG_BANK_SEL_RESERVED_3 0x08
#define ICM_REG_BANK_SEL_BIT_4      0x10  // 0: Select USER BANK 0.
#define ICM_REG_BANK_SEL_BIT_5      0x20  // 1: Select USER BANK 1.
                                          // 2: Select USER BANK 2.
                                          // 3: Select USER BANK 3.
#define ICM_REG_BANK_SEL_RESERVED_6 0x40
#define ICM_REG_BANK_SEL_RESERVED_7 0x80

/*
 * USER BANK 1
 */
#define ICM_USER_BANK_1             0x10
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
#define ICM_REG_BANK_SEL_RESERVED_0 0x01
#define ICM_REG_BANK_SEL_RESERVED_1 0x02
#define ICM_REG_BANK_SEL_RESERVED_2 0x04
#define ICM_REG_BANK_SEL_RESERVED_3 0x08
#define ICM_REG_BANK_SEL_BIT_4      0x10 // 0: Select USER BANK 0.
#define ICM_REG_BANK_SEL_BIT_5      0x20 // 1: Select USER BANK 1.
                                         // 2: Select USER BANK 2.
                                         // 3: Select USER BANK 3.
#define ICM_REG_BANK_SEL_RESERVED_6 0x40
#define ICM_REG_BANK_SEL_RESERVED_7 0x80

/*
 * USER BANK 2
 */
#define ICM_USER_BANK_2 0x20
#define ICM_GYRO_SMPLRT_DIV         0x00  // R/W GYRO_SMPLRT_DIV[7:0]
#define ICM_GYRO_CONFIG_1 0x01  // R/W
#define ICM_GYRO_CONFIG_1_RESERVED_7 0x80
#define ICM_GYRO_CONFIG_1_RESERVED_6 0x40
#define ICM_GYRO_GYRO_DLPFCFG_5      0x20
#define ICM_GYRO_GYRO_DLPFCFG_4      0x10
#define ICM_GYRO_GYRO_DLPFCFG_3      0x08
#define ICM_GYRO_GYRO_FS_SEL_2       0x4
#define ICM_GYRO_GYRO_FS_SEL_1       0x2
#define ICM_GYRO_GYRO_FCHOICE        0x1
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
#define ICM_ACCEL_CONFIG_RESERVED_7      0x80
#define ICM_ACCEL_CONFIG_RESERVED_6      0x40
#define ICM_ACCEL_CONFIG_ACCEL_DLPFCFG_2 0x20
#define ICM_ACCEL_CONFIG_ACCEL_DLPFCFG_1 0x10
#define ICM_ACCEL_CONFIG_ACCEL_DLPFCFG_0 0x08
#define ICM_ACCEL_CONFIG_ACCEL_FS_SEL_1  0x04 // 00 = ±250 dps
#define ICM_ACCEL_CONFIG_ACCEL_FS_SEL_0  0x02 // 01= ±500 dps
                                              // 10 = ±1000 dps
                                              // 11 = ±2000 dps
#define ICM_ACCEL_CONFIG_ACCEL_FCHOICE   0x01

#define ICM_ACCEL_CONFIG_2          0x15  // R/W - AX_ST_EN_R EGAY_ST_EN_REG AZ_ST_EN_REG DEC3_CFG[1:0]
#define ICM_FSYNC_CONFIG            0x52  // R/W DELAY_TIME_EN - WOF_DEGLITCH_EN WOF_EDGE_INT EXT_SYNC_SET[3:0]
#define ICM_TEMP_CONFIG             0x53  // R/W - TEMP_DLPFCFG[2:0]
#define ICM_MOD_CTRL_USR            0x54  // R/W - REG_LP_DMP_EN

/*
 * USER BANK 3
 */
#define ICM_USER_BANK_3             0x30
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

/*
 * Start Register map for AK09916
 */

/*
 * Read-only register
 */
#define ICM_AK_WIA1  0x00 // 0 1 0 0 1 0 0 0
#define ICM_AK_WIA2  0x01 // 0 0 0 0 1 0 0 1
#define ICM_AK_RSV1  0x02 // RSV17 RSV16 RSV15 RSV14 RSV13 RSV12 RSV11 RSV10
#define ICM_AK_RSV2  0x03 // RSV27 RSV26 RSV25 RSV24 RSV23 RSV22 RSV21 RSV20
#define ICM_AK_ST1   0x10 // 0 0 0 0 0 0 DOR DRDY
#define ICM_AK_HXL   0x11 // HXL HX7 HX6 HX5 HX4 HX3 HX2 HX1 HX0
#define ICM_AK_HXH   0x12 // HXH HX15 HX14 HX13 HX12 HX11 HX10 HX9 HX8
#define ICM_AK_HYL   0x13 // HYL HY7 HY6 HY5 HY4 HY3 HY2 HY1 HY0
#define ICM_AK_HYH   0x14 // HYH HY15 HY14 HY13 HY12 HY11 HY10 HY9 HY8
#define ICM_AK_HZL   0x15 // HZL HZ7 HZ6 HZ5 HZ4 HZ3 HZ2 HZ1 HZ0
#define ICM_AK_HZH   0x16 // HZH HZ15 HZ14 HZ13 HZ12 HZ11 HZ10 HZ9 HZ8
#define ICM_AK_TMPS  0x17 // TMPS 0 0 0 0 0 0 0 0
#define ICM_AK_ST2   0x18 // ST2 0 RSV30 RSV29 RSV28 HOFL 0 0 0

/*
 * Read/Write register
 */
#define ICM_AK_CNTL1 0x30 // CNTL1 0 0 0 0 0 0 0 0
#define ICM_AK_CNTL2 0x31 // 0 0 0 MODE4 MODE3 MODE2 MODE1 MODE0
#define ICM_AK_CNTL3 0x32 // 0 0 0 0 0 0 0 SRST

/*
 * End Register map for AK09916
 */

#define IMU1 0
#define IMU2 1
#define IMU3 2

#define TX_NO_STOP       1 // TX transfer will not end with a stop condition.
#define TX_MAY_STOP      0 // TX transfer may end with a stop condition.

#define REC_BUF_LEN 54     /*  3 IMU:r * 3 sensors * 3 axis * 2 bytes */

typedef struct
{
    uint8_t device_id;      /* 0x30 - 0x32, 0x40 - 0x42, 0x50 - 0x52, 0x60 - 0x62 */
    uint8_t packet_length; /* 14-242 bytes (2 + n*12) */
    union
    {
	float    f;
	uint16_t u;
    } acc_x;
    union
    {
	float    f;
	uint16_t u;
    } acc_y;
    union
    {
	float    f;
	uint16_t u;
    } acc_z;
    union
    {
	float    f;
	uint16_t u;
    } gyr_x;
    union
    {
	float    f;
	int16_t  u;
    } gyr_y;
    union
    {
	float    f;
	uint16_t u;
    } gyr_z;
    union
    {
	float    f;
	uint16_t u;
    } mag_x;
    union
    {
	float    f;
	uint16_t u;
    } mag_y;
    union
    {
	float    f;
	uint16_t u;
    } mag_z;
    union
    {
	float    f;
	uint16_t  u;
    } eul_x;
    union
    {
	float    f;
	uint16_t u;
    } eul_y;
    union
    {
	float    f;
	uint16_t  u;
    } eul_z;
} icm_imu_data_t;

/**
 * @brief Initialize both I2C channels
 * Used ports SDA and SCL must been configured first
 */
extern ret_code_t io_i2cInit();

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
extern nrfx_err_t io_i2cTx(uint8_t Address, char *data, uint16_t Len, uint8_t noStop);

/**
 * @brief I2C Read
 *
 * @param Address = address byte from the device
 * @param dest = received data
 * @param Len = required length of the received data
 * @return = zero mean not all required data ready
 */
extern nrfx_err_t io_i2cRx(uint8_t Address, char *dest, uint16_t Len);

/**
 * @brief Initiate I2C
 *
 * @param -
 * @return -
 */
extern void icmInitI2c(void);

/**
 * @brief Initiate Device
 *
 * @param -
 * @return -
 */
extern void icmInitiateDevice(uint8_t imu_addr);

/**
 * @brief Initiate Icm20948
 *
 * @param -
 * @return -
 */
extern void icmInitiateIcm20948(uint8_t imu_addr);

/**
 * @brief Initiate AK09916
 *
 * @param -
 * @return -
 */
extern void icmInitiateAk09916(uint8_t imu_addr);

/**
 * @brief Read chip ID
 *
 * @param -
 * @return -
 */
extern void icmReadChipId(uint8_t imu_addr);

/**
 * @brief Read magnetometer data
 *
 * @param -
 * @return -
 */
extern void readMagnReg (uint8_t imu_addr, uint8_t reg, uint8_t length);

/**
 * @brief Write magnetometer data
 *
 * @param -
 * @return -
 */
extern void writeMagnReg(uint8_t imu_addr, char reg, char data);

/**
 * @brief Reset of the device
 *
 * @param -
 * @return -
 */
void icmDeviceReset(uint8_t imu_addr);

/**
 * @brief Read temperature of thermometer in device
 *
 * @param -
 * @return -
 */
void icmReadTempData(uint8_t imu_addr);

/**
 * @brief Read accelerometer raw data from device
 *
 * @param int16_t * destination
 * @return -
 */
void readAccelData(uint8_t imu_addr);

/**
 * @brief Read gyroscope raw data from device
 *
 * @param int16_t * destination
 * @return -
 */
void readGyroData(uint8_t imu_addr);

/**
 * @brief Read magnetometer raw data from device
 *
 * @param int16_t * destination
 * @return -
 */
void readMagnData(uint8_t imu_addr);

/**
 * @brief Get IMU data from driver
 *
 * @param icm_imu_data *imu_data
 * @return -
 */
void icm_get_imu_data(icm_imu_data_t *imu_data);

#endif /* _ICM20948_DRV_H_ */
