#include "radar_hal.h"
#include <string.h>

#include <nrf_drv_spi.h>
#include <nrf_gpio.h>
#include <sdk_config.h>
#define NRF_LOG_MODULE_NAME "APP"
#include <nrf_log.h>
#include <nrf_log_ctrl.h>



static const nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(0);
static volatile bool spi_xfer_done;
/***
static uint8_t       m_tx_buf[255];
static uint8_t       m_rx_buf[255];

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.\r\n");
    if (m_rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received: \r\n");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}
*/
int radar_hal_init(radar_handle_t ** radar_handle, void* instance_memory)
{

    uint32_t err_code;
    int status = XT_SUCCESS;
    radar_handle_t * radar_handle_local = (radar_handle_t *)instance_memory;
    memset(radar_handle_local, 0, sizeof(radar_handle_t));
    radar_handle_local->radar_id = 0;


    *radar_handle = radar_handle_local;

    nrf_drv_spi_config_t config = NRF_DRV_SPI_DEFAULT_CONFIG;
    config.frequency = NRF_DRV_SPI_FREQ_1M;
    config.mode      = NRF_DRV_SPI_MODE_3;
    config.bit_order = NRF_DRV_SPI_BIT_ORDER_LSB_FIRST;
    //lagt til dette jeg
    spi_config.ss_pin   = 22;
    spi_config.miso_pin = 24;
    spi_config.mosi_pin = 23;
    spi_config.sck_pin  = 25;
    err_code = nrf_drv_spi_init(&spi_instance, &config, NULL, NULL);
    if (err_code != NRF_SUCCESS)
    {
        // Initialization failed. Take recovery action.
    }
    return status;

}

int radar_hal_get_instance_size(void)
{
    int size = sizeof(radar_handle_t);
    //size += xtio_spi_get_instance_size();
    return size;
}

uint32_t radar_hal_pin_set_enable(radar_handle_t * radar_handle, uint8_t value)
{
  //might have to configure GPIO to drive signal
    if(value == 1)
    {
        nrf_gpio_cfg_output(X4_ENABLE_PIN);
        nrf_gpio_pin_set(X4_ENABLE_PIN);
    }
    else if(value == 0)
    {
        nrf_gpio_cfg_output(X4_ENABLE_PIN);
        nrf_gpio_pin_clear(X4_ENABLE_PIN);
    }
    else
    {
        return XT_ERROR;
    }
    return XT_SUCCESS;
}

uint32_t radar_hal_spi_write(radar_handle_t * radar_handle, uint8_t* data, uint32_t length)
{
    if (0 == length)
    {
        return XT_SUCCESS;
    }
    if (NULL == data)
    {
        return XT_ERROR;
    }
    nrf_drv_spi_transfer(&spi_instance, data, length, NULL,0);
    return XT_SUCCESS;
}

uint32_t radar_hal_spi_read(radar_handle_t * radar_handle, uint8_t* data, uint32_t length)
{
    if (0 == length)
    {
        return XT_SUCCESS;
    }
    if (NULL == data)
    {
        return XT_ERROR;
    }
    //xtio_spi_handle_t * spi_handle = (xtio_spi_handle_t *)radar_handle->spi_handle;
    //return spi_handle->spi_write_read(spi_handle, NULL, 0, data, length);
    return nrf_drv_spi_transfer(&spi_instance, NULL, 0, data,length);
}

uint32_t radar_hal_spi_write_read(radar_handle_t * radar_handle, uint8_t* wdata, uint32_t wlength, uint8_t* rdata, uint32_t rlength)
{
    if ((0 == wlength) && (0 == rlength))
    {
      return XT_SUCCESS;
    }
    if ((NULL == wdata) || (NULL == rdata))
    {
      return XT_ERROR;
    }
    return nrf_drv_spi_transfer(&spi_instance, wdata, wlength, rdata,rlength);
}
