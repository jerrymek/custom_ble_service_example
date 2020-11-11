/**
 * Copyright  2020 PreCure ApS
 *
 * All Rights Reserved
 */
#ifndef _ADS1298_DRV_H_
#define _ADS1298_DRV_H_

#define NUMBER_OF_CHANNELS (8)
#define REC_BUF_LEN 27     // status 3 bytes + (8 channels * 3 bytes)

typedef enum
{
    NORMAL_ELECTRODE_INPUT = 0x0,
    DEVICE_NOISE_MEASUREMENTS,
    RLD_MEASUREMENTS,
    SUPPLY_MEASUREMENT,
    TEMPERATURE_SENSOR,
    TEST_SIGNAL,
    RLD_DRP,
    RLD_DRN
}  channel_input_e;

ret_code_t ads_init_spi(void);
ret_code_t ads_read_ID(void);
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
ret_code_t ads_read_basic_data(uint8_t *rx_buf);
ret_code_t ads_capture_ADC_data(uint8_t *rx_buf);
ret_code_t ads_set_channel_x(channel_input_e chan_set);
ret_code_t ads_set_config(uint8_t config1_settings,
			  uint8_t config2_settings);
ret_code_t ads_start_measurerment(uint8_t *rx_buf);
void ads_print_rec_data(uint8_t *rx_buf);

#endif // EOF- ads1298_drv.h
