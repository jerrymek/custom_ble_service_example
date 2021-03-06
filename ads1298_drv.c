/**
 * Copyright  2020 PreCure ApS
 *
 * All Rights Reserved
 */

/**@brief
 * @description
 */
#include <string.h>
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_drv_spi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "log_support.h"
#include "nrfx_spi.h"
#include "ads1298_drv.h"

/**
 * COMMAND DESCRIPTION FIRST BYTE, SECOND BYTE
 * See 
 */

/**
 * @brief Register Map
 */
#define ADS_ID     0x0
#define CONFIG1    0x1 // 06 HR DAISY_EN CLK_EN 0 0 DR2 DR1 DR0
#define CONFIG2    0x2 // 40 0 0 WCT_CHOP INT_TEST 0 TEST_AMP TEST_FREQ1 TEST_FREQ0 RLD_LOFF_
#define CONFIG3    0x3 // 40 PD_REFBUF 1 VREF_4V RLD_MEAS RLDREF_INT PD_RLD RLD_STAT SENS
#define VLEAD_OFF_ 0x4 // LOFF 00 COMP_TH2 COMP_TH1 COMP_TH0 ILEAD_OFF1 ILEAD_OFF0 FLEAD_OFF1 FLEAD_OFF0 EN
// CHANNEL-SPECIFIC SETTINGS
#define CH1SET 0x5 // 00 PD1 GAIN12 GAIN11 GAIN10 0 MUX12 MUX11 MUX10
#define CH2SET 0x6 // 00 PD2 GAIN22 GAIN21 GAIN20 0 MUX22 MUX21 MUX20
#define CH3SET 0x7 // 00 PD3 GAIN32 GAIN31 GAIN30 0 MUX32 MUX31 MUX30
#define CH4SET 0x8 // 00 PD4 GAIN42 GAIN41 GAIN40 0 MUX42 MUX41 MUX40
#define CH5SET 0x9 // (1) 00 PD5 GAIN52 GAIN51 GAIN50 0 MUX52 MUX51 MUX50
#define CH6SET 0xA // (1) 00 PD6 GAIN62 GAIN61 GAIN60 0 MUX62 MUX61 MUX60
#define CH7SET 0xB // (1) 00 PD7 GAIN72 GAIN71 GAIN70 0 MUX72 MUX71 MUX70
#define CH8SET 0xC // (1) 00 PD8 GAIN82 GAIN81 GAIN80 0 MUX82 MUX81 MUX80
#define RLD_SENSP 0xD // (2) 00 RLD8P(1) RLD7P(1) RLD6P(1) RLD5P(1) RLD4P RLD3P RLD2P RLD1P
#define RLD_SENSN 0xE // (2) 00 RLD8N(1) RLD7N(1) RLD6N(1) RLD5N(1) RLD4N RLD3N RLD2N RLD1N
#define LOFF_SENSP 0xF // (2) 00 LOFF8P LOFF7P LOFF6P LOFF5P LOFF4P LOFF3P LOFF2P LOFF1P
#define LOFF_SENSN 0x10 // (2) 00 LOFF8N LOFF7N LOFF6N LOFF5N LOFF4N LOFF3N LOFF2N LOFF1N
#define LOFF_FLIP 0x11 // 00 LOFF_FLIP8 LOFF_FLIP7 LOFF_FLIP6 LOFF_FLIP5 LOFF_FLIP4 LOFF_FLIP3 LOFF_FLIP2 LOFF_FLIP1
// LEAD-OFF STATUS REGISTERS (READ-ONLY REGISTERS)
#define LOFF_STATP 0x12 // 00 IN8P_OFF IN7P_OFF IN6P_OFF IN5P_OFF IN4P_OFF IN3P_OFF IN2P_OFF IN1P_OFF
#define LOFF_STATN 0x13 // 00 IN8N_OFF IN7N_OFF IN6N_OFF IN5N_OFF IN4N_OFF IN3N_OFF IN2N_OFF IN1N_OFF
// GPIO AND OTHER REGISTERS
#define GPIO 0x14 // 0F GPIOD4 GPIOD3 GPIOD2 GPIOD1 GPIOC4 GPIOC3 GPIOC2 GPIOC1
#define PACE 0x15 // 00 0 0 0 PACEE1 PACEE0 PACEO1 PACEO0 PD_PACE RESP_ RESP_MOD_
#define RESP 0x16 // 00 1 RESP_PH2 RESP_PH1 RESP_PH0 RESP_CTRL1 RESP_CTRL0 DEMOD_EN1 EN1 SINGLE_ WCT_TO_ PD_LOFF_
#define CONFIG4 0x17 // 00 RESP_FREQ2 RESP_FREQ1 RESP_FREQ0 0 0 SHOT RLD COMP
#define WCT1 0x18 // 00 aVF_CH6 aVL_CH5 aVR_CH7 avR_CH4 PD_WCTA WCTA2 WCTA1 WCTA0
#define WCT2 0x19 // 00 PD_WCTC PD_WCTB WCTB2 WCTB1 WCTB0 WCTC2 WCTC1 WCTC0

/**
 * SYSTEM COMMANDS
 */
uint8_t ADS_WAKEUP  = (0x02); // Wakeup from standby mode
uint8_t ADS_STANDBY = (0x04); // Enter standby mode
uint8_t ADS_RESET   = (0x06); // Reset the device
uint8_t ADS_START   = (0x08); // Start/restart (synchronize) conversions
uint8_t ADS_STOP    = (0x0a); // Stop conversion

/**
 * DATA READ COMMANDS
 */
uint8_t ADS_RDATAC  = (0x10); // Enable Read Data Continuous mode (default at power up).
uint8_t ADS_SDATAC  = (0x11); // Stop Read Data Continuously mode.
uint8_t ADS_RDATA   = (0x12); // Read data by command. supports multiple read back.

/**
 * REGISTER READ COMMANDS
 *
 * When in RDATAC mode, the RREG command is ignored.
 */
#define ADS_RREG    (0x2)  // Read n nnnn registers starting at address r rrrr.
                           // 001r rrrr 000n nnnn (2)
#define ADS_WREG    (0x4)  // Write n nnnn registers starting at address r rrrr.
                           // 010r rrrr 000n nnnn (2)

#define PIN_LOW     (0x0)
#define PIN_HIGH    (0x1)

//!< SPI instance index.
#define SPI_INSTANCE (0)

//!< SPI instance.
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(0);
//!< Flag used to indicate that SPI instance completed the transfer.
static volatile bool spi_xfer_done;


/*@brief
 *@description
 *  rrrrr Starting register address for read/write opcodes.
 *  nnnnn Number of registers to be read/written – 1.
 */
typedef struct
{
    unsigned char op_code:3;
    unsigned char rrrrr:5;
} ads_reg_byte0_t;

typedef struct
{
    unsigned char dummy:3;
    unsigned char nnnnn:5;
} ads_reg_byte1_t;

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
//Todo:    NRF_LOG_DEBUG("%s(%d) Transfer completed.", __FILENAME__, __LINE__);
}

#define GENERAL_FAILURE 0xffff  // Todo: no code for initiating err_code to unsuccessful.
ret_code_t ads_send_command(uint8_t *tx_buf, uint8_t tx_len, uint8_t *rx_buf, uint8_t rx_len)
{
    ret_code_t err_code = GENERAL_FAILURE;
    spi_xfer_done = false;
    nrf_gpio_pin_clear(SPI_SS_PIN);
    if ( rx_len > 0 )
    {
    	while (nrf_gpio_pin_read(SPI_DRDY_PIN) != PIN_LOW);
    }
    while ((err_code = nrf_drv_spi_transfer(&spi,
					    tx_buf,
					    tx_len,
					    rx_buf,
					    rx_len)) == NRF_ERROR_BUSY);
    MY_ERROR_CHECK(err_code);
    while (!spi_xfer_done)
    {
        __WFE();
    }
    nrf_gpio_pin_set(SPI_SS_PIN);
    if (rx_len > 0)
    {
        printf("%d, ", rx_len);
	for (uint8_t i = 0; i < rx_len; i++)
	{
	    printf("%02x", rx_buf[i]);
	}
	printf("\n");
    }
    return err_code;
}

/**
 * @brief Initate the ADS chip for Basic Data Capture
 * @description See 10.1.1 Setting the Device for Basic Data Capture in
 * in reference 4 in the Description of the Lab Setup.
 */
ret_code_t ads_init_spi(void)
{
    ret_code_t err_code = GENERAL_FAILURE;
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
 
    nrf_gpio_cfg_output(SPI_SS_PIN);                       // Configure Slave Select pin.
    nrf_gpio_cfg_input(SPI_DRDY_PIN, NRF_GPIO_PIN_PULLUP); // Configure Data Ready pin.
    nrf_gpio_cfg_output(ADS_RESET_PIN);                    // Configure ADS Reset Pin.

    nrf_gpio_pin_set(ADS_RESET_PIN);    // 2) Set the RESET pin high
    nrf_delay_ms(100);                  // 3) Wait after powerup until reset
    nrf_gpio_pin_clear(ADS_RESET_PIN);  // 4) Set RESET pin low for a minimum of 2 * tCLK
    nrf_delay_ms(10);                   // 5) wait > 2 * tCLK
    nrf_gpio_pin_set(ADS_RESET_PIN);    // 6) Setting RESET pin high now enables the digital portion of the ADS1298
//    nrf_delay_ms(2500);                 // 7) Wait > 18 * tCLK Before starting to use the device
//    NRF_LOG_DEBUG("%s(%d) Reset of ADS1298 done.",
//		  __FILENAME__, __LINE__);
    printf("%s(%d) Reset of ADS1298 done.\n",
	   __FILENAME__, __LINE__);

    spi_config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED; // Manual control of the CS pin, set the Slave Select pin to not used.
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    spi_config.frequency = NRF_DRV_SPI_FREQ_500K; // Ensure less than 4 clk cycles for ADS1292/8}
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    spi_config.mode      = NRF_DRV_SPI_MODE_1;
    err_code = nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL);
    MY_ERROR_CHECK(err_code);
//    NRF_LOG_DEBUG("%s(%d) SPI initiated.",
//		  __FILENAME__, __LINE__);
    printf("%s(%d) SPI initiated.\n",
	   __FILENAME__, __LINE__);

    return err_code;
}

ret_code_t ads_read_ID(void)
{
    ret_code_t err_code = GENERAL_FAILURE;
    uint8_t tx_buf[3] = { 0, 0, 0 };

    // Device Wakes Up in RDATAC Mode, so Send SDATAC Command so Registers can be Written
    NRF_LOG_DEBUG("%s(%d) ADS_SDATAC.", __FILENAME__, __LINE__);
    MY_ERROR_CHECK(ads_send_command(&ADS_SDATAC, 1, NULL, 0));

    // WREG CONFIG1
    tx_buf[0] = ((ADS_WREG & 0x7) << 5 | (CONFIG1 & 0x1f));
    tx_buf[1] = 0;
    tx_buf[2] = 0xA0;
    NRF_LOG_DEBUG("%s(%d) 0x%x 0x%x 0x%x",
    		  __FILENAME__, __LINE__,
    		  tx_buf[0], tx_buf[1], tx_buf[2]);
    err_code = ads_send_command(tx_buf, 3, NULL, 0);
    MY_ERROR_CHECK(err_code);
                                                   //               RREG = 2       ADS_ID = 0
                                                   //                   r rrrr dum n nnnn
    tx_buf[0] = (ADS_RREG << 5 | (ADS_ID & 0x1f)); // 0x2 << 5 => 0x010 0 0000 000 0 0000
    tx_buf[1] = 0;
    tx_buf[2] = 0;
    uint8_t idcode = 0;
    uint8_t idlen = 1;
    NRF_LOG_DEBUG("%s(%d) 0x%x 0x%x 0x%x",
		  __FILENAME__, __LINE__,
		  tx_buf[0], tx_buf[1], tx_buf[2]);
    err_code = ads_send_command(tx_buf, 2, &idcode, idlen);

    NRF_LOG_DEBUG("%s(%d) RREG ADS_ID, ID code = 0x%x, ID len = 0x%x.",
		  __FILENAME__, __LINE__,
		  idcode, idlen);

    return err_code;
}

ret_code_t ads_set_channel_x(channel_input_e chan_set)
{
    ret_code_t err_code = GENERAL_FAILURE;
    uint8_t tx_reg[3] = { 0, 0, 0 };

    for (int i = 0; i < NUMBER_OF_CHANNELS; i++)
    {
	// Set All Channels to Input Short WREG CHnSET chan_set
	tx_reg[0] = (ADS_WREG << 5 | ((i + 1) & 0x1f));
	tx_reg[1] = 0;
	tx_reg[2] = chan_set;
	err_code = ads_send_command(tx_reg, 3, NULL, 0);
	MY_ERROR_CHECK(err_code);
    }
    NRF_LOG_DEBUG("%s(%d) WREG CHnSET 0x%x 0x%x 0x%x for all eight channels",
		  __FILENAME__, __LINE__, tx_reg[0], tx_reg[1], tx_reg[2]);

    return err_code;
}

ret_code_t ads_set_config(uint8_t config1_settings, uint8_t config2_settings)
{
    ret_code_t err_code = GENERAL_FAILURE;
    uint8_t tx_reg[3] = { 0, 0, 0 };

    // Device Wakes Up in RDATAC Mode, so Send
    // SDATAC Command so Registers can be Written
    NRF_LOG_DEBUG("%s(%d) ADS_SDATAC.", __FILENAME__, __LINE__);
    err_code = ads_send_command(&ADS_SDATAC, 1, NULL, 0);
    MY_ERROR_CHECK(err_code);

    NRF_LOG_DEBUG("%s(%d) ADS_RESET(0x%x).", __FILENAME__, __LINE__, ADS_RESET);
    err_code = ads_send_command(&ADS_RESET, 1, NULL, 0);
    MY_ERROR_CHECK(err_code);

    // Set Device in High Resolution Mode and DR = f /1024 WREG CONFIG1 0x86
    tx_reg[0] = (ADS_WREG << 5 | (CONFIG1 & 0x1f));
    tx_reg[1] = 0;
    tx_reg[2] = config1_settings;
    NRF_LOG_DEBUG("%s(%d) WREG CONFIG1 0x%x.", __FILENAME__, __LINE__, tx_reg[2]);
    err_code = ads_send_command(tx_reg, 3, NULL, 0);
    MY_ERROR_CHECK(err_code);
    // WREG CONFIG2 0x00
    tx_reg[0] = (ADS_WREG << 5 | (CONFIG2 & 0x1f));
    tx_reg[1] = 0;
    tx_reg[2] = config2_settings;
    NRF_LOG_DEBUG("%s(%d) WREG CONFIG2 0x%x.", __FILENAME__, __LINE__, tx_reg[2]);
    err_code = ads_send_command(tx_reg, 3, NULL, 0);
    MY_ERROR_CHECK(err_code);

    // WREG CONFIG2 0x00
    tx_reg[0] = (ADS_WREG << 5 | (CONFIG4 & 0x1f));
    tx_reg[1] = 0;
    tx_reg[2] = 0x20; // 0010 0000
    NRF_LOG_DEBUG("%s(%d) WREG CONFIG2 0x%x.", __FILENAME__, __LINE__, tx_reg[2]);
    err_code = ads_send_command(tx_reg, 3, NULL, 0);
    MY_ERROR_CHECK(err_code);

    return err_code;
}

ret_code_t ads_start_measurerment(uint8_t *rx_buf)
{
    ret_code_t err_code = GENERAL_FAILURE;

    // Start Conversion
    // After This Point !DRDY Should Toggle at
    // fclk /4096
    err_code = ads_send_command(&ADS_START, 1, NULL, 0);
    MY_ERROR_CHECK(err_code);

    // Put the Device Back in RDATAC Mode
    err_code = ads_send_command(&ADS_RDATAC, 1, NULL, 0);
    MY_ERROR_CHECK(err_code);

 //   while (nrf_gpio_pin_read(SPI_DRDY_PIN) != PIN_LOW);

    // SDATAC Command so Registers can be Written
    err_code = ads_send_command(&ADS_SDATAC, 1, rx_buf, REC_BUF_LEN);
        printf("Test\n");
    MY_ERROR_CHECK(err_code);

    return err_code;
}

void ads_print_rec_data(uint8_t *rx_buf)
{
    NRF_LOG_DEBUG("Status bytes: 0x%02x%02x%02x", rx_buf[0], rx_buf[1], rx_buf[2]);
    NRF_LOG_DEBUG("Channel 1:    0x%02x%02x%02x", rx_buf[3], rx_buf[4], rx_buf[5]);
    NRF_LOG_DEBUG("Channel 2:    0x%02x%02x%02x", rx_buf[6], rx_buf[7], rx_buf[8]);
    NRF_LOG_DEBUG("Channel 3:    0x%02x%02x%02x", rx_buf[9], rx_buf[10], rx_buf[11]);
    NRF_LOG_DEBUG("Channel 4:    0x%02x%02x%02x", rx_buf[12], rx_buf[13], rx_buf[14]);
    NRF_LOG_DEBUG("Channel 5:    0x%02x%02x%02x", rx_buf[15], rx_buf[16], rx_buf[17]);
    NRF_LOG_DEBUG("Channel 6:    0x%02x%02x%02x", rx_buf[18], rx_buf[19], rx_buf[20]);
    NRF_LOG_DEBUG("Channel 7:    0x%02x%02x%02x", rx_buf[21], rx_buf[22], rx_buf[23]);
    NRF_LOG_DEBUG("Channel 8:    0x%02x%02x%02x", rx_buf[24], rx_buf[25], rx_buf[26]);
}

ret_code_t ads_hello_world(void)
{
    ret_code_t err_code = GENERAL_FAILURE;
    uint8_t tx_reg[3] = { 0, 0, 0 };
    uint8_t rx_buf[REC_BUF_LEN] = { 0, 0, 0,
				    0, 0, 0,
				    0, 0, 0,
				    0, 0, 0,
				    0, 0, 0,
				    0, 0, 0,
				    0, 0, 0,
				    0, 0, 0,
				    0, 0, 0};
 
    NRF_LOG_DEBUG("Start of Hello World");

    err_code = ads_set_config(0xA0, 0x00);
    
    err_code = ads_set_channel_x(DEVICE_NOISE_MEASUREMENTS); // Device Noise Measurements

    err_code = ads_start_measurerment(rx_buf);

//    ads_print_rec_data(rx_buf);

 /* ---- */

    for (uint8_t i = 0; i < REC_BUF_LEN; i++)
    {
        rx_buf[i] = 0;
    }

    err_code = ads_set_config(0xA0, 0x00);

    // WREG CONFIG2 0x00
    tx_reg[0] = (ADS_WREG << 5 | (CONFIG4 & 0x1f));
    tx_reg[1] = 0;
    tx_reg[2] = 0x20; // 0010 0000
    NRF_LOG_DEBUG("%s(%d) WREG CONFIG2 0x%x.", __FILENAME__, __LINE__, tx_reg[2]);
    err_code = ads_send_command(tx_reg, 3, NULL, 0);
    MY_ERROR_CHECK(err_code);
    
    err_code = ads_set_channel_x(DEVICE_NOISE_MEASUREMENTS);

    err_code = ads_start_measurerment(rx_buf);

    ads_print_rec_data(rx_buf);

    NRF_LOG_DEBUG("End of Hello World");

    return err_code;
}

ret_code_t ads_capture_ADC_data(uint8_t *rx_buf)
{
    ret_code_t err_code = GENERAL_FAILURE;
    uint8_t tx_reg[3] = { 0, 0, 0 };

    err_code = ads_set_config(0xA0, 0x00);

    err_code = ads_set_channel_x(DEVICE_NOISE_MEASUREMENTS);

    err_code = ads_start_measurerment(rx_buf);

    //    ads_print_rec_data(rx_buf);

    return (err_code);

}

ret_code_t ads_read_basic_data(uint8_t *rx_buf)
{
    ret_code_t err_code = GENERAL_FAILURE;

    err_code = ads_set_config(0xA0, 0x00);

    err_code = ads_set_channel_x(DEVICE_NOISE_MEASUREMENTS);

    err_code = ads_start_measurerment(rx_buf);

//    ads_print_rec_data(rx_buf);

    return (err_code);

}
 
