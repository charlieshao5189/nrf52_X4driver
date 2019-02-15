#include "radar_hal.h"
#include "xep_hal.h"
#include <string.h>

#include <nrf_drv_spi.h>
#include <nrf_drv_timer.h>
#include <nrf_gpio.h>
#include <sdk_config.h>
//#define NRF_LOG_MODULE_NAME "APP"
#include <nrf_log.h>
#include <nrf_log_ctrl.h>

static const nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(0);

//static void burst_setup()
//{
//    uint32_t spi_end_evt;
//    uint32_t timer_count_task;
//    uint32_t timer_cc_event;
//    ret_code_t err_code;       
//
//    //Configure timer
//    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
//    timer_cfg.mode = NRF_TIMER_MODE_COUNTER;
//    err_code = nrf_drv_timer_init(&timer, &timer_cfg, timer_event_handler);
//    APP_ERROR_CHECK(err_code);
//
//    // Compare event after 4 transmissions
//    nrf_drv_timer_extended_compare(&timer, NRF_TIMER_CC_CHANNEL0, 4, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
//
//    timer_count_task = nrf_drv_timer_task_address_get(&timer, NRF_TIMER_TASK_COUNT);
//    timer_cc_event = nrf_drv_timer_event_address_get(&timer,  NRF_TIMER_EVENT_COMPARE0);
//    NRF_LOG_INFO("timer configured\n");
//
//    // allocate PPI channels for hardware
//    err_code = nrf_drv_ppi_channel_alloc(&ppi_channel_spi);
//    APP_ERROR_CHECK(err_code);
//    err_code = nrf_drv_ppi_channel_alloc(&ppi_channel_timer);
//    APP_ERROR_CHECK(err_code);
//    NRF_LOG_INFO("ppi allocated\n");
//
//    spi_end_evt = nrf_drv_spi_end_event_get(&spi);
//
//    // Configure the PPI to count the trasnsactions on the TIMER
//    err_code = nrf_drv_ppi_channel_assign(ppi_channel_spi, spi_end_evt, timer_count_task);
//    APP_ERROR_CHECK(err_code);
//
//    // Configure another PPI to stop the SPI when 4 transactions have been completed
//    err_code = nrf_drv_ppi_channel_assign(ppi_channel_timer, timer_cc_event, NRF_SPIM_TASK_STOP);
//    APP_ERROR_CHECK(err_code);
//    NRF_LOG_INFO("ppi configured\n");
//}

int radar_hal_init(radar_handle_t **radar_handle, void *instance_memory) {

  uint32_t err_code;
  int status = XT_SUCCESS;
  radar_handle_t *radar_handle_local = (radar_handle_t *)instance_memory;
  memset(radar_handle_local, 0, sizeof(radar_handle_t));
  radar_handle_local->radar_id = 0;

  *radar_handle = radar_handle_local;

  nrf_gpio_cfg_output(X4_ENABLE_PIN);
  nrf_gpio_cfg_output(X4_SPI_SS);

  nrf_gpio_pin_set(X4_SPI_SS);
  nrf_gpio_cfg_input(X4_GPIO_INT, GPIO_PIN_CNF_PULL_Pulldown);

  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  spi_config.miso_pin = 24;
  spi_config.mosi_pin = 23;
  spi_config.sck_pin = 25;
  spi_config.irq_priority = 2;
  spi_config.frequency = SPIM_FREQUENCY_FREQUENCY_M2;

  //err_code = nrf_drv_spi_init(&spi_instance, &spi_config, spi_event_handler, NULL);
  err_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
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
  
//  if(rlength + wlength <= 255){
    uint8_t * temp_buff = calloc(wlength+rlength, sizeof (uint8_t));
    nrf_gpio_pin_clear(X4_SPI_SS);
    err_code = nrf_drv_spi_transfer(&spi_instance, wdata, wlength, temp_buff, rlength + wlength);
    nrf_gpio_pin_set(X4_SPI_SS);
    memcpy(rdata, temp_buff+wlength, rlength);
    free(temp_buff);
//  }


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

  nrf_gpio_pin_clear(X4_SPI_SS);
  err_code = nrf_drv_spi_transfer(&spi_instance, data, length, NULL, 0);
  nrf_gpio_pin_set(X4_SPI_SS);

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