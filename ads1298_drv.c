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
#define ADS_REG_ID     0x0

/**
 * SYSTEM COMMANDS
 */
#define ADS_WAKEUP  (0x02) // Wakeup from standby mode
#define ADS_STANDBY (0x04) // Enter standby mode
#define ADS_RESET   (0x06) // Reset the device
#define ADS_START   (0x08) // Start/restart (synchronize) conversions
#define ADS_STOP    (0x0a) // Stop conversion

/**
 * DATA READ COMMANDS
 */
#define ADS_RDATAC  (0x10) // Enable Read Data Continuous mode (default at power up).
#define ADS_SDATAC  (0x11) // Stop Read Data Continuously mode.
#define ADS_RDATA   (0x12) // Read data by command. supports multiple read back.

/**
 * REGISTER READ COMMANDS
 *
 * When in RDATAC mode, the RREG command is ignored.
 */
#define ADS_RREG    (0x2)  // Read n nnnn registers starting at address r rrrr.
                           // 001r rrrr 000n nnnn (2)
#define ADS_WREG    (0x4)  // Write n nnnn registers starting at address r rrrr.
                           // 010r rrrr 000n nnnn (2)

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


#define TEST_STRING "0"
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_DEBUG("%s(%d) Transfer completed.", __FILENAME__, __LINE__);
    if (m_rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}

/**
 * @brief Initate the ADS chip for Basic Data Capture
 * @description See 10.1.1 Setting the Device for Basic Data Capture in
 * in reference 4 in the Description of the Lab Setup.
 */
ads_rc_e ads_init_spi(void)
{
    ads_rc_e rc = ADS_RESULT_NOT_OK;
    ret_code_t err_code = 0xffff; // Todo: no code for initiating err_code to unsuccessful.

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    // Ensure less than 4 clk cycles for ADS1292/8}
    spi_config.frequency = NRF_DRV_SPI_FREQ_500K; // Ensure less than 4 clk cycles for ADS1292/8}
    spi_config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED; // Manual control of the CS pin, set the Slave Select pin to not used.
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    err_code = nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL);
    MY_ERROR_CHECK(err_code);

    nrf_gpio_cfg_output(SPI_SS_PIN);

    return err_code;
}

ads_rc_e ads_hello_world(void)
{
    ads_rc_e rc = ADS_RESULT_NOT_OK;
    ret_code_t err_code = 0; // Todo: no code for initiating err_code to unsuccessful.
 
    /*
     * Stop the continuos read mode to allow use of RREG.
     */
    uint8_t cmd = ADS_SDATAC;
    // Reset rx buffer and transfer done flag
    spi_xfer_done = false;
    NRF_LOG_DEBUG("%s(%d) ADS_SDATAC.", __FILENAME__, __LINE__);
    nrf_gpio_pin_clear(SPI_SS_PIN);
    err_code = nrf_drv_spi_transfer(&spi, &cmd, 1, NULL, 0);
    while (!spi_xfer_done)
    {
        __WFE();
    }
    spi_xfer_done = false;
    nrf_gpio_pin_set(SPI_SS_PIN);
    MY_ERROR_CHECK(err_code);

    /*
     * Stop the ADC conversion.
     */
    //cmd = ADS_STOP;
    //nrf_gpio_pin_clear(SPI_SS_PIN);
    //err_code = nrf_drv_spi_transfer(&spi, &cmd, 1, NULL, 0);
    //nrf_gpio_pin_set(SPI_SS_PIN);
    //MY_ERROR_LOG(err_code);

    /*
     * Send command to read the HW ID from the ADS chip.
     */
    uint8_t rx_buf[64] = {0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0,
                          0,0,0,0,0,0,0,0};
    uint8_t buf[2] = {ADS_RREG, 0};
    NRF_LOG_DEBUG("%s(%d) ADS_RREG.", __FILENAME__, __LINE__);
    nrf_gpio_pin_clear(SPI_SS_PIN);
    err_code = nrf_drv_spi_transfer(&spi, buf, 2, rx_buf, 64);
    while (!spi_xfer_done)
    {
        __WFE();
    }
    spi_xfer_done = false;
    nrf_gpio_pin_set(SPI_SS_PIN);
    NRF_LOG_DEBUG("%s(%d) rx_buf[0] = 0x%x", __FILENAME__, __LINE__, rx_buf);
    MY_ERROR_CHECK(err_code);


    /*
     * Read data returned from ADS.
     */
    //uint8_t *rx_buf;
    //memset(rx_buf, 0, 1);
    //nrf_gpio_pin_clear(SPI_SS_PIN);
    //err_code = nrf_drv_spi_transfer(&spi, NULL, 0, rx_buf, 1);
    //while (!spi_xfer_done)
    //{
    //    __WFE();
    //}
    //spi_xfer_done = false;
    //nrf_gpio_pin_set(SPI_SS_PIN);
    //NRF_LOG_DEBUG("%s(%d) rx_buf[0] = 0x%x", __FILENAME__, __LINE__, rx_buf);
    //MY_ERROR_CHECK(err_code);
    return rc;
}
