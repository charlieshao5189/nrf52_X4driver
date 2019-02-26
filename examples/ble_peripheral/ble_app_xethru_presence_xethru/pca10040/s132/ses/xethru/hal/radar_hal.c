#include "radar_hal.h"
#include "xep_hal.h"
#include <string.h>

#include <nrf_drv_timer.h>
#include <nrfx_timer.h>
#include <nrf_drv_ppi.h>
#include <nrf_drv_spi.h>
#include <nrf_gpio.h>
#include <nrfx_gpiote.h>
#include <sdk_config.h>
//#define NRF_LOG_MODULE_NAME "APP"
#include <nrf_log.h>
#include <nrf_log_ctrl.h>

const nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(0);
const nrf_drv_timer_t timer_instance = NRF_DRV_TIMER_INSTANCE(1);

/**************SPI burst*******start*****************/

// PPI resrources
nrf_ppi_channel_t ppi_channel_spi;
nrf_ppi_channel_t ppi_channel_timer;
static volatile bool burst_completed = false;
static volatile bool spi_xfer_done = false;  
static volatile bool last_spi_read = false; 

uint8_t spi_rx_end_number;
volatile uint8_t spi_rx_end_count;


 nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;


static void spi_event_handler(nrf_drv_spi_evt_t const *p_event) {
  if (p_event->type == NRF_DRV_SPI_EVENT_DONE) {
    spi_xfer_done = true;
    //nrf_gpio_pin_set(X4_SPI_SS);
    //burst_completed = true;
  } else {
    NRF_LOG_INFO("Wrong Event\n");
    // Something is wrong
  }
}

static volatile int remaining_to_transfer = 0;
static volatile uint8_t* read_ptr = NULL;

static void general_spi_event_handler(nrf_drv_spi_evt_t const *p_event) {
  if (p_event->type == NRF_DRV_SPI_EVENT_DONE) {
     //NRF_LOG_INFO("spi_rx_end_count:%d",spi_rx_end_count);
     if (remaining_to_transfer > 0)
     {
        int current_transfer = remaining_to_transfer;
        if (current_transfer > 255)
            current_transfer = 255;
        nrf_drv_spi_xfer_desc_t xfer = NRF_DRV_SPI_XFER_TRX(NULL, 0, read_ptr, (uint8_t)current_transfer);
        read_ptr += current_transfer;
        remaining_to_transfer -= current_transfer;
        nrf_drv_spi_xfer(&spi_instance, &xfer, NULL);
     } else {  
       nrf_gpio_pin_set(X4_SPI_SS);
       burst_completed = true;
     }
  } else{
    NRF_LOG_INFO("Wrong Event\n");
    // Something is wrong
  }
}

/**
 * @brief Work-around for transmitting 1 byte with SPIM.
 */
void setup_workaround_for_ftpan_58(bool enable) // uint32_t ppi_channel)
{
  static nrf_ppi_channel_t spi_workaround_ppi_channel;
  if (enable)
  {
    nrfx_ppi_channel_alloc(&spi_workaround_ppi_channel);
    nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_TOGGLE(true);
    nrfx_gpiote_in_init(NRF_SPIM0->PSEL.SCK,
                               &in_config,
                               NULL);
    nrfx_gpiote_in_event_enable(NRF_SPIM0->PSEL.SCK, false);

    // Stop the spim instance when SCK toggles.
    nrfx_ppi_channel_assign(spi_workaround_ppi_channel, nrfx_gpiote_in_event_addr_get(NRF_SPIM0->PSEL.SCK), (uint32_t)&NRF_SPIM0->TASKS_STOP);
    nrf_ppi_channel_enable(spi_workaround_ppi_channel);

    // The spim instance cannot be stopped mid-byte, so it will finish
    // transmitting the first byte and then stop. Effectively ensuring
    // that only 1 byte is transmitted.
    } else
    {
      nrf_ppi_channel_disable(spi_workaround_ppi_channel);
      nrfx_ppi_channel_free(spi_workaround_ppi_channel);
      nrfx_gpiote_in_uninit(NRF_SPIM0->PSEL.SCK);
    }
}

static uint8_t general_spi_write_read(uint8_t *wdata, uint32_t wlength, uint8_t *rdata, uint32_t rlength){
   ret_code_t err_code;

    remaining_to_transfer =  rlength;
    read_ptr = rdata;

    if (rlength == 1)
      setup_workaround_for_ftpan_58(true);

   nrf_drv_spi_xfer_desc_t xfer = NRF_DRV_SPI_XFER_TRX(wdata, wlength, NULL, 0);

   burst_completed = false;
   nrf_gpio_pin_clear(X4_SPI_SS);
   nrf_drv_spi_xfer(&spi_instance, &xfer, NULL);
   while (!burst_completed) {
    __WFE();
  }
  burst_completed = false;

    if (rlength == 1)
      setup_workaround_for_ftpan_58(false);
}

uint8_t spi_init(void) {
  ret_code_t err_code;

  err_code = nrf_drv_ppi_init();
  APP_ERROR_CHECK(err_code);
  //NRF_LOG_INFO("ppi initialised\n");

  nrf_gpio_cfg_output(X4_SPI_SS);
  nrf_gpio_pin_set(X4_SPI_SS);

  spi_config.miso_pin = 24;
  spi_config.mosi_pin = 23;
  spi_config.sck_pin = 25;
  spi_config.irq_priority = 2;
  spi_config.frequency = SPIM_FREQUENCY_FREQUENCY_M2;

  //err_code = nrf_drv_spi_init(&spi_instance, &spi_config, spi_event_handler,NULL);
  err_code = nrf_drv_spi_init(&spi_instance, &spi_config, general_spi_event_handler,NULL);
  APP_ERROR_CHECK(err_code);
  //NRF_LOG_INFO("SPI0 Master initialised.\r\n");

  // Reset rx buffer and transfer done flag
  //    memset(m_rx_buf, 0, BUFF_LENGTH);
  //    memset(m_tx_buf, 0, BUFF_LENGTH);
  spi_xfer_done = false;

  //    // To read the Device ID (0xD1)
  //    u8 data = 0;
  //    bmi160_bus_read(0, 0x0, &data, 1);
  //    NRF_LOG_INFO("DEVICE ID (0xD1): %#01x\r\n", data);

  return XT_SUCCESS;
}



/**************SPI burst*********end*****************/

int radar_hal_init(radar_handle_t **radar_handle, void *instance_memory) {

  uint32_t err_code = NRF_SUCCESS;
  int status = XT_SUCCESS;
  radar_handle_t *radar_handle_local = (radar_handle_t *)instance_memory;
  memset(radar_handle_local, 0, sizeof(radar_handle_t));
  radar_handle_local->radar_id = 0;

  *radar_handle = radar_handle_local;

   nrf_gpio_cfg_output(X4_ENABLE_PIN);
  //  nrf_gpio_cfg_output(X4_SPI_SS);
  //
  //  nrf_gpio_pin_set(X4_SPI_SS);
    nrf_gpio_cfg_input(X4_GPIO_INT, GPIO_PIN_CNF_PULL_Pulldown);
  //
  //  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  //  spi_config.miso_pin = 24;
  //  spi_config.mosi_pin = 23;
  //  spi_config.sck_pin = 25;
  //  spi_config.irq_priority = 2;
  //  spi_config.frequency = SPIM_FREQUENCY_FREQUENCY_M2;
  //
  //  err_code = nrf_drv_spi_init(&spi_instance, &spi_config, spi_event_handler, NULL);
  //  err_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);

  spi_init();
  if (err_code != NRF_SUCCESS) {
    status = XT_ERROR;
  }
  return status;
}

int radar_hal_get_instance_size(void) {
  int size = sizeof(radar_handle_t);
  //size += xtio_spi_get_instance_size();
  return size;
}

uint32_t radar_hal_pin_set_enable(radar_handle_t *radar_handle, uint8_t value) {
  //might have to configure GPIO to drive signal
  if (value == 1) {
    nrf_gpio_pin_set(X4_ENABLE_PIN);
  } else if (value == 0) {
    nrf_gpio_pin_clear(X4_ENABLE_PIN);
  } else {
    return XT_ERROR;
  }
  return XT_SUCCESS;
}

uint32_t radar_hal_spi_write_read(radar_handle_t *radar_handle, uint8_t *wdata, uint32_t wlength, uint8_t *rdata, uint32_t rlength) {
  uint32_t err_code = XT_SUCCESS;

  if ((0 == wlength) && (0 == rlength)) {
    return XT_SUCCESS;
  }
  if ((NULL == wdata) || (NULL == rdata)) {
    return XT_ERROR;
  }

  general_spi_write_read(wdata, wlength, rdata, rlength);

  if (err_code == XT_SUCCESS) {
    return XT_SUCCESS;
  } else {
    return XT_ERROR;
  }
}

uint32_t radar_hal_spi_write(radar_handle_t *radar_handle, uint8_t *data, uint32_t length) {
  uint32_t err_code = XT_SUCCESS;

  if (0 == length) {
  }
  if (NULL == data) {
    return XT_ERROR;
  }

  general_spi_write_read(data, length, NULL, 0);

  if (err_code == XT_SUCCESS) {
    return XT_SUCCESS;
  } else {
    return XT_ERROR;
  }
}

uint32_t radar_hal_spi_read(radar_handle_t *radar_handle, uint8_t *data, uint32_t length) {
  uint32_t err_code = XT_SUCCESS;

  if (0 == length) {
    return XT_SUCCESS;
  }
  if (NULL == data) {
    return XT_ERROR;
  }
  return radar_hal_spi_write_read(radar_handle, NULL, 0, data, length);
}