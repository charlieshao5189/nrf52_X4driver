#include "radar_hal.h"
#include "xep_hal.h"
#include <string.h>

#include <nrf_drv_timer.h>
#include <nrfx_timer.h>
#include <nrf_drv_ppi.h>
#include <nrf_drv_spi.h>
#include <nrf_gpio.h>
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


static void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            if(last_spi_read == false){
            // Configure short between spi end event and spi start task
             nrf_drv_timer_extended_compare(&timer_instance, NRF_TIMER_CC_CHANNEL0, 1, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
             nrf_spim_shorts_disable(spi_instance.u.spim.p_reg, NRF_SPIM_SHORT_END_START_MASK);
             NRF_SPIM0->RXD.MAXCNT = 172;
             // Compare event after rx_end_count transmissions
              //burst_setup(1);
              //burst_transfer_enable();
              last_spi_read = true;
            }else{
              burst_transfer_disable();
            }
            break;

        default:
            //Do nothing.
            break;
    }
}

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

void burst_setup(uint8_t rx_end_count) {
  uint32_t spi_end_evt;
  uint32_t timer_count_task;
  uint32_t timer_cc_event;
  ret_code_t err_code;

  //Configure timer
  nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
  timer_cfg.mode = NRF_TIMER_MODE_COUNTER;
  err_code = nrfx_timer_init(&timer_instance, &timer_cfg, timer_event_handler); 
  APP_ERROR_CHECK(err_code);

  // Compare event after rx_end_count transmissions
  nrf_drv_timer_extended_compare(&timer_instance, NRF_TIMER_CC_CHANNEL0, rx_end_count, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
  //nrf_drv_timer_extended_compare(&timer_instance, NRF_TIMER_CC_CHANNEL0, rx_end_count, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
  
  timer_count_task = nrf_drv_timer_task_address_get(&timer_instance, NRF_TIMER_TASK_COUNT);
  timer_cc_event = nrf_drv_timer_event_address_get(&timer_instance, NRF_TIMER_EVENT_COMPARE0);
  //NRF_LOG_INFO("timer configured\n");

  // allocate PPI channels for hardware
  err_code = nrf_drv_ppi_channel_alloc(&ppi_channel_spi);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_ppi_channel_alloc(&ppi_channel_timer);
  APP_ERROR_CHECK(err_code);
  //NRF_LOG_INFO("ppi allocated\n");

  spi_end_evt = nrf_drv_spi_end_event_get(&spi_instance);

  // Configure the PPI to count the trasnsactions on the TIMER
  err_code = nrf_drv_ppi_channel_assign(ppi_channel_spi, spi_end_evt, timer_count_task);
  APP_ERROR_CHECK(err_code);

  // Configure another PPI to stop the SPI when 4 transactions have been completed
  err_code = nrf_drv_ppi_channel_assign(ppi_channel_timer, timer_cc_event, NRF_SPIM_TASK_STOP);
  APP_ERROR_CHECK(err_code);
  //NRF_LOG_INFO("ppi configured\n");
}

static void burst_transfer_enable() {
  ret_code_t err_code;

  burst_completed = false;
  last_spi_read = false; 

  err_code = nrf_drv_ppi_channel_enable(ppi_channel_spi);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_ppi_channel_enable(ppi_channel_timer);
  APP_ERROR_CHECK(err_code);

  nrf_drv_timer_enable(&timer_instance);

  nrf_gpio_pin_clear(X4_SPI_SS);
}

void burst_transfer_disable() {
  ret_code_t err_code;

  nrf_gpio_pin_set(X4_SPI_SS);
  err_code = nrf_drv_ppi_channel_disable(ppi_channel_spi);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_ppi_channel_disable(ppi_channel_timer);
  APP_ERROR_CHECK(err_code);

  nrf_drv_timer_disable(&timer_instance);
  nrfx_timer_uninit(&timer_instance);  
  err_code = nrf_drv_ppi_channel_free(ppi_channel_spi);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_ppi_channel_free(ppi_channel_timer);
  APP_ERROR_CHECK(err_code);

  burst_completed = true;
}


uint8_t burst_read(uint8_t *wdata, uint32_t wlength, uint8_t *rdata, uint32_t rlength){
  ret_code_t err_code;
  uint8_t rx_end_count = (rlength+wlength-1)/255;
  nrf_drv_spi_xfer_desc_t xfer = NRF_DRV_SPI_XFER_TRX(wdata, wlength, rdata, 255);
    // Configure short between spi end event and spi start task
  nrf_spim_shorts_enable(spi_instance.u.spim.p_reg, NRF_SPIM_SHORT_END_START_MASK);
  uint32_t flags = NRF_DRV_SPI_FLAG_HOLD_XFER |
                   NRF_DRV_SPI_FLAG_RX_POSTINC |
                   NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER;
  nrf_drv_spi_uninit(&spi_instance); 
  err_code = nrf_drv_spi_init(&spi_instance, &spi_config, spi_event_handler,NULL);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_spi_xfer(&spi_instance, &xfer, flags);
  APP_ERROR_CHECK(err_code);

  burst_setup(rx_end_count);
  
  burst_transfer_enable();

  nrf_spim_task_trigger(spi_instance.u.spim.p_reg, NRF_SPIM_TASK_START);

  while (!burst_completed) {
    __WFE();
  }
  spi_xfer_done = false;
  burst_completed = false;

  return XT_SUCCESS;
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
        read_ptr += current_transfer;
        nrf_drv_spi_xfer_desc_t xfer = NRF_DRV_SPI_XFER_TRX(NULL, 0, read_ptr, current_transfer);
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
static uint8_t general_spi_write_read(uint8_t *wdata, uint32_t wlength, uint8_t *rdata, uint32_t rlength){
   ret_code_t err_code;

    remaining_to_transfer =  rlength;

    read_ptr = rdata;
    int current_transfer = remaining_to_transfer;
    if (current_transfer > 255)
        current_transfer = 255;

   nrf_drv_spi_xfer_desc_t xfer = NRF_DRV_SPI_XFER_TRX(wdata, wlength, read_ptr, current_transfer);
   remaining_to_transfer -= current_transfer;

   burst_completed = false;
   nrf_gpio_pin_clear(X4_SPI_SS);
   nrf_drv_spi_xfer(&spi_instance, &xfer, NULL);
   while (!burst_completed) {
    __WFE();
  }
  burst_completed = false;
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

  uint32_t err_code;
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

    general_spi_write_read(wdata,wlength,rdata-1,rlength+wlength);

//  if (rlength + wlength <= 255) {
//  nrf_drv_spi_uninit(&spi_instance); 
//  err_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL,NULL);
//  APP_ERROR_CHECK(err_code);
//
//    uint8_t *temp_buff = calloc(wlength + rlength, sizeof(uint8_t));
//    nrf_gpio_pin_clear(X4_SPI_SS);
//    err_code = nrf_drv_spi_transfer(&spi_instance, wdata, wlength, temp_buff, rlength + wlength);
//    nrf_gpio_pin_set(X4_SPI_SS);
//    memcpy(rdata, temp_buff + wlength, rlength);
//    free(temp_buff);
//
//  } else {
//    burst_read(wdata,wlength,rdata-1,rlength+wlength);
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

    general_spi_write_read(data, length, NULL, 0);
//  nrf_drv_spi_uninit(&spi_instance); 
//  err_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL,NULL);
//  APP_ERROR_CHECK(err_code);
//  nrf_gpio_pin_clear(X4_SPI_SS);
//  err_code = nrf_drv_spi_transfer(&spi_instance, data, length, NULL, 0);
//  nrf_gpio_pin_set(X4_SPI_SS);

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