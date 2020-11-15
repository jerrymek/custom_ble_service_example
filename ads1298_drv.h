/**
 * Copyright  2020 PreCure ApS
 *
 * All Rights Reserved
 */
#ifndef _ADS1298_DRV_H_
#define _ADS1298_DRV_H_

void ads_init_gpio_pins(void);
void ads_power_up_sequence(void);
void ads_init_spi(void);
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
