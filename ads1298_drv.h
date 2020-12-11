/**
 * Copyright  2020 PreCure ApS
 *
 * All Rights Reserved
 */
#ifndef _ADS1298_DRV_H_
#define _ADS1298_DRV_H_

/*
 * Configure ADS inputs.
 */
#define NORMAL_ELECTRODE_INPUT 0x00 // Bits 0, 1 and 2
#define INPUT_SHORTED          0x01
#define RLD_MEAS_BIT_FOR_RLD   0x02
#define MVDD_FOR_SUPPLY_MEAS   0x03
#define TEMPERATURE_SENSOR     0x04
#define TEST_SIGNAL            0x05
#define POSITIVE_ELECTRODE     0x06
#define NEGATIVE_ELECTRODE     0x07

#define POWER_DOWN             0x80 // 0=Normal operation, 1=Power down
#define PGA_GAIN_2             0x40 // if bits 4, 5 and 6 are not set, i.e. PGA gain = 6.
#define PGA_GAIN_1             0x20
#define PGA_GAIN_0             0x10

typedef struct
{
    uint16_t chan1;
    uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;

} ads_emg_data_t;

/*
 * For an input range of +/-800mV, the least significant bit
 * from the ADS has a weight of (1600 mV/(2^24-1) bits)=96 nV.
 */
#define ADS_LSB_WEIGHT 96;

void ads_init_gpio_pins(void);
void ads_power_up_sequence(void);
void ads_configure_measurment(uint8_t configuration);
void ads_configure_normal_input_measurment(void);
void ads_configure_shorted_input_measurment(void);
void ads_read_adc_data(void);
void ads_init_spi(void);
void ads_get_channel_data(ads_emg_data_t *emg_data);
uint8_t ads_read_ID(uint8_t expected_ID);
ret_code_t ads_hello_world (void);
ret_code_t ads_wakeup (void);
ret_code_t ads_standby (void);
ret_code_t ads_reset (void);
ret_code_t ads_start (void);
ret_code_t ads_stop (void);
ret_code_t ads_start_contineus_mode (void);
ret_code_t ads_stop_contineus_mode (void);
ret_code_t ads_read_data (void);
ret_code_t ads_read_registers (uint8_t start,
			     uint8_t length);
ret_code_t ads_write_registers (uint8_t start,
			      uint8_t length,
			      uint8_t *result);
ret_code_t ads_setting_the_device (void);
ret_code_t ads_basic_data_capture (void);

#endif // EOF- ads1298_drv.h
