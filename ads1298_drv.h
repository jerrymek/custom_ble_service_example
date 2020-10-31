/**
 * Copyright  2020 PreCure ApS
 *
 * All Rights Reserved
 */
#ifndef _ADS1298_DRV_H_
#define _ADS1298_DRV_H_

typedef enum
{
    ADS_RESULT_OK = 0,
    ADS_RESULT_NOT_OK = 1
} ads_rc_e;

ads_rc_e ads_init_spi(void);
ads_rc_e ads_hello_world (void);
ads_rc_e ads_wakeup (void);
ads_rc_e ads_standby (void);
ads_rc_e ads_reset (void);
ads_rc_e ads_start (void);
ads_rc_e ads_stop (void);
ads_rc_e ads_start_contineus_mode (void);
ads_rc_e ads_stop_contineus_mode (void);
ads_rc_e ads_read_data (void);
ads_rc_e ads_read_registers (uint8_t start,
			     uint8_t length);
ads_rc_e ads_write_registers (uint8_t start,
			      uint8_t length,
			      uint8_t *result);
ads_rc_e ads_setting_the_device (void);
ads_rc_e ads_basic_data_capture (void);

#endif // EOF- ads1298_drv.h
