#include "taskRadar.h"
#include "app_error.h"
#include "boards.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_ppi.h"
#include "nrf_log.h"
#include "radar_hal.h"
#include "x4driver.h"
#include "xep_hal.h"
#include <string.h> 

volatile xtx4driver_errors_t x4_initialize_status = XEP_ERROR_X4DRIVER_UNINITIALIZED;
X4Driver_t *x4driver = NULL;

#define DEBUG 0

typedef struct
{
  //TaskHandle_t radar_task_handle;
  radar_handle_t *radar_handle; // Some info separating different radar chips on the same module.
} XepRadarX4DriverUserReference_t;

typedef struct
{
  //XepDispatch_t* dispatch;
  X4Driver_t *x4driver;
} RadarTaskParameters_t;

void x4driver_GPIO_init(void) {
  // done by radar_hal_int
}

void x4driver_spi_init(void) {
  // done by radar_hal_int
}

//void spi_event_handler(nrf_drv_spi_evt_t const *p_event,
//    void *p_context) {
//  spi_xfer_done = true;
//  NRF_LOG_INFO("Transfer completed.");
//  if (register_read_buffer != 0x55) {
//    NRF_LOG_INFO(" Received:");
//    NRF_LOG_HEXDUMP_INFO(&register_read_buffer, 1);
//  }
//}

static uint32_t bin_count = 0;
static uint32_t fdata_count = 0;
static uint8_t down_conversion_enabled = 0;
static volatile bool first_frame = true;
static uint32_t frame_counter = 0;
static uint8_t data_frame_bytes[6000];
static float32_t *data_frame_normolized;


void x4driver_data_ready(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  nrf_drv_gpiote_out_toggle(BSP_LED_0); //blink LED1 with FPS frequency
  uint32_t status = XEP_ERROR_X4DRIVER_OK;

 //read bytes directly
  if (true == first_frame) {
    first_frame = false;
    x4driver_get_frame_bin_count(x4driver, &bin_count);
    x4driver_get_downconversion(x4driver, &down_conversion_enabled);
    fdata_count = bin_count;
    if (down_conversion_enabled == 1) {
      fdata_count = bin_count * 2;
    }
    //data_frame_bytes = (uint8_t *) malloc(x4driver->frame_read_size);
  }
  status = x4driver_read_frame_bytes(x4driver, &frame_counter, data_frame_bytes, x4driver->frame_read_size);

  if (XEP_ERROR_X4DRIVER_OK == status) {
    NRF_LOG_INFO("\n x4 frame data ready! \r\n");

  } else {
    NRF_LOG_INFO("fail to get x4 frame data errorcode:%d! \r\n", status);
  }

  printf("Size:%d,New Frame Data Normalized(%d){\r\n", x4driver->frame_read_size, frame_counter);
  for (uint32_t i = 0; i < x4driver->frame_read_size; i=i+1) {
    //printf("[%d]:%X, ",i, data_frame_bytes[i]);
    printf("%X, ",data_frame_bytes[i]);
  }
  printf("}\r\n");

//    if (true == first_frame) {
//    first_frame = false;
//    x4driver_get_frame_bin_count(x4driver, &bin_count);
//    x4driver_get_downconversion(x4driver, &down_conversion_enabled);
//    fdata_count = bin_count;
//    if (down_conversion_enabled == 1) {
//      fdata_count = bin_count * 2;
//    }
//    data_frame_normolized = (float32_t *) calloc(0,fdata_count);
//    //data_frame_normolized = (float32_t *) malloc(fdata_count);
//    }
//
//    status = x4driver_read_frame_normalized(x4driver,&frame_counter,data_frame_normolized,fdata_count);
//
//    if(XEP_ERROR_X4DRIVER_OK == status)
//    {
//        NRF_LOG_INFO("x4 frame data ready! \n");
//
//    }
//    else
//    {
//        NRF_LOG_INFO("fail to get x4 frame data errorcode:%d! \n", status);
//    }
//
//    printf("Size:%d,New Frame Data Normolized(%d){\n",fdata_count,frame_counter);
//    for(uint32_t i=0; i<fdata_count; i++)
//    {
//       //printf("[%d]:%f, ",i, data_frame_normolized[i]);
//       printf("%f, ", data_frame_normolized[i]);
//        //NRF_LOG_INFO("[%d]:" NRF_LOG_FLOAT_MARKER,i,NRF_LOG_FLOAT(data_frame_normolized[i]) );
//        //NRF_LOG_INFO(NRF_LOG_FLOAT_MARKER,NRF_LOG_FLOAT(data_frame_normolized[i]) );
//        //NRF_LOG_INFO(NRF_LOG_FLOAT_MARKER,NRF_LOG_FLOAT(0.168f) );
//    }
//    printf("}\n");

}

static uint32_t x4driver_callback_take_sem(void *sem, uint32_t timeout) {
  //x4driver_mutex.lock();
  return 1;
}

static void x4driver_callback_give_sem(void *sem) {
  //  x4driver_mutex.unlock();
}

static uint32_t x4driver_callback_pin_set_enable(void *user_reference, uint8_t value) {
  XepRadarX4DriverUserReference_t *x4driver_user_reference = (XepRadarX4DriverUserReference_t *)user_reference;
  int status = radar_hal_pin_set_enable(x4driver_user_reference->radar_handle, value);
  return status;
}

static uint32_t x4driver_callback_spi_write(void *user_reference, uint8_t *data, uint32_t length) {
  XepRadarX4DriverUserReference_t *x4driver_user_reference = (XepRadarX4DriverUserReference_t *)user_reference;
  return radar_hal_spi_write(x4driver_user_reference->radar_handle, data, length);
}
static uint32_t x4driver_callback_spi_read(void *user_reference, uint8_t *data, uint32_t length) {
  XepRadarX4DriverUserReference_t *x4driver_user_reference = (XepRadarX4DriverUserReference_t *)user_reference;
  return radar_hal_spi_read(x4driver_user_reference->radar_handle, data, length);
}

static uint32_t x4driver_callback_spi_write_read(void *user_reference, uint8_t *wdata, uint32_t wlength, uint8_t *rdata, uint32_t rlength) {
  XepRadarX4DriverUserReference_t *x4driver_user_reference = (XepRadarX4DriverUserReference_t *)user_reference;
  return radar_hal_spi_write_read(x4driver_user_reference->radar_handle, wdata, wlength, rdata, rlength);
}

static void x4driver_callback_wait_us(uint32_t us) {
  nrf_delay_us(us);
}

void x4driver_enable_ISR(void *user_reference, uint32_t enable) {
  ret_code_t err_code;

  if (enable == 1) {
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
    err_code = nrf_drv_gpiote_out_init(BSP_LED_0, &out_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN;
    err_code = nrf_drv_gpiote_in_init(X4_GPIO_INT, &in_config, x4driver_data_ready);
    nrf_drv_gpiote_in_event_enable(X4_GPIO_INT, true);
  } else {
    nrf_gpio_cfg_output(X4_GPIO_INT);
  }
}

uint32_t task_radar_init(X4Driver_t **x4driver) {
  //x4driver_GPIO_init();
  //x4driver_spi_init();

  XepRadarX4DriverUserReference_t *x4driver_user_reference = (XepRadarX4DriverUserReference_t *)malloc(sizeof(XepRadarX4DriverUserReference_t));
  memset(x4driver_user_reference, 0, sizeof(XepRadarX4DriverUserReference_t));

  void *radar_hal_memory = malloc(radar_hal_get_instance_size());
  int status = radar_hal_init(&(x4driver_user_reference->radar_handle), radar_hal_memory);

#ifdef DEBUG
  if (status == XT_SUCCESS) {
    printf("radar_hal_init success\n");
  } else {
    NRF_LOG_INFO("radar_hal_init unknown situation\n");
  }
#endif // DEBUG

  //! [X4Driver Platform Dependencies]

  // X4Driver lock mechanism, including methods for locking and unlocking.
  X4DriverLock_t lock;
  //lock.object = (void *)&x4driver_mutex;
  lock.object = NULL;
  lock.lock = x4driver_callback_take_sem;
  lock.unlock = x4driver_callback_give_sem;

  // X4Driver timer for generating sweep FPS on MCU. Not used when sweep FPS is generated on X4.
  //    uint32_t timer_id_sweep = 2;
  X4DriverTimer_t timer_sweep;
  //    timer_sweep.object = xTimerCreate("X4Driver_sweep_timer", 1000 / portTICK_PERIOD_MS, pdTRUE, (void*)timer_id_sweep, x4driver_timer_sweep_timeout);
  //    timer_sweep.configure = x4driver_timer_set_timer_timeout_frequency;

  // X4Driver timer used for driver action timeout.
  //    uint32_t timer_id_action = 3;
  X4DriverTimer_t timer_action;
  //    timer_action.object = xTimerCreate("X4Driver_action_timer", 1000 / portTICK_PERIOD_MS, pdTRUE, (void*)timer_id_action, x4driver_timer_action_timeout);
  //	timer_action.configure = x4driver_timer_set_timer_timeout_frequency;

  // X4Driver callback methods.
  X4DriverCallbacks_t x4driver_callbacks;

  x4driver_callbacks.pin_set_enable = x4driver_callback_pin_set_enable; // X4 ENABLE pin
  x4driver_callbacks.spi_read = x4driver_callback_spi_read;             // SPI read method
  x4driver_callbacks.spi_write = x4driver_callback_spi_write;           // SPI write method
  x4driver_callbacks.spi_write_read = x4driver_callback_spi_write_read; // SPI write and read method
  x4driver_callbacks.wait_us = x4driver_callback_wait_us;               // Delay method
                                                                        //  x4driver_callbacks.notify_data_ready = x4driver_notify_data_ready;      // Notification when radar data is ready to read
                                                                        //  x4driver_callbacks.trigger_sweep = x4driver_trigger_sweep_pin;          // Method to set X4 sweep trigger pin
  x4driver_callbacks.enable_data_ready_isr = x4driver_enable_ISR;       // Control data ready notification ISR

  void *x4driver_instance_memory = malloc(x4driver_get_instance_size()); //pvPortMalloc(x4driver_get_instance_size());
  //x4driver_create(x4driver, x4driver_instance_memory, &x4driver_callbacks,&lock,&timer_sweep,&timer_action, (void*)x4driver_user_reference);
  x4driver_create(x4driver, x4driver_instance_memory, &x4driver_callbacks, &lock, &timer_sweep, &timer_action, x4driver_user_reference);

#ifdef DEBUG
  if (status == XEP_ERROR_X4DRIVER_OK) {
    NRF_LOG_INFO("x4driver_create success\n");
  } else {
    NRF_LOG_INFO("x4driver_create unknown situation\n");
  }
#endif // DEBUG

  RadarTaskParameters_t *task_parameters = (RadarTaskParameters_t *)malloc(sizeof(RadarTaskParameters_t));
  //task_parameters->dispatch = dispatch;
  task_parameters->x4driver = *x4driver;

  task_parameters->x4driver->com_buffer_size = 192 * 32;
  //task_parameters->x4driver->spi_buffer_size = 4609;// baseband 188*2*6= 2256   rf 1536*3= 4608
  task_parameters->x4driver->com_buffer = (uint8_t *)malloc(task_parameters->x4driver->com_buffer_size);
  if ((((uint32_t)task_parameters->x4driver->com_buffer) % 32) != 0) {
    int alignment_diff = 32 - (((uint32_t)task_parameters->x4driver->com_buffer) % 32);
    task_parameters->x4driver->com_buffer += alignment_diff;
    task_parameters->x4driver->com_buffer_size -= alignment_diff;
  }
  task_parameters->x4driver->com_buffer_size -= task_parameters->x4driver->com_buffer_size % 32;

  //    xTaskCreate(task_radar, (const char * const) "Radar", TASK_RADAR_STACK_SIZE, (void*)task_parameters, TASK_RADAR_PRIORITY, &h_task_radar);
  //    x4driver_user_reference->radar_task_handle = h_task_radar;

  return XT_SUCCESS;
}

int taskRadar(void) {
  NRF_LOG_INFO("task_radar start!\n");

  uint32_t status = 0;
  //uint8_t* data_frame;

  //initialize radar task

  status = task_radar_init(&x4driver);

#ifdef DEBUG
  if (status == XT_SUCCESS) {
    NRF_LOG_INFO("task_radar_init success\n");
  } else if (status == XT_ERROR) {
    NRF_LOG_INFO("task_radar_init failure\n");
  } else {
    NRF_LOG_INFO("task_radar_init unknown situation\n");
  }
#endif // DEBUG

  xtx4driver_errors_t tmp_status = (xtx4driver_errors_t)x4driver_init(x4driver);

#ifdef DEBUG
  if (tmp_status == XEP_ERROR_X4DRIVER_OK) {
    NRF_LOG_INFO("x4driver_init success\n");
  } else {
    NRF_LOG_INFO("x4driver_init unknown situation\n");
  }
#endif // DEBUG

  status = x4driver_set_sweep_trigger_control(x4driver, SWEEP_TRIGGER_X4); // By default let sweep trigger control done by X4
#ifdef DEBUG
  if (status == XEP_ERROR_X4DRIVER_OK) {
    NRF_LOG_INFO("x4driver_set_sweep_trigger_control success\n");
  } else {
    NRF_LOG_INFO("x4driver_set_sweep_trigger_control unknown situation\n");
  }
#endif // DEBUG

  //    x4_initialize_status = tmp_status;

  status = x4driver_set_dac_min(x4driver, 949);
  if (status != XEP_ERROR_X4DRIVER_OK) {
#ifdef DEBUG
    NRF_LOG_INFO("Error setting dac minimum\n");
    NRF_LOG_INFO("Error code=%d\n", status);
#endif
    return 1;
  }
#ifdef DEBUG
  NRF_LOG_INFO("x4driver_set_dac_min success\n");
#endif
  status = x4driver_set_dac_max(x4driver, 1100);
  if (status != XEP_ERROR_X4DRIVER_OK) {
#ifdef DEBUG
    NRF_LOG_INFO("Error setting dac maximum\n");
    NRF_LOG_INFO("Error code=%d\n", status);
#endif
    return 1;
  }
#ifdef DEBUG
  NRF_LOG_INFO("x4driver_set_dac_max success\n");
#endif
  status = x4driver_set_iterations(x4driver, 32);
  if (status != XEP_ERROR_X4DRIVER_OK) {
#ifdef DEBUG
    NRF_LOG_INFO("Error in x4driver_set_iterations\n");
    NRF_LOG_INFO("Error code=%d\n", status);
#endif
    return 1;
  }
#ifdef DEBUG
  NRF_LOG_INFO("x4driver_set_iterations success\n");
#endif
  status = x4driver_set_pulses_per_step(x4driver, 140);
  if (status != XEP_ERROR_X4DRIVER_OK) {
#ifdef DEBUG
    NRF_LOG_INFO("Error in x4driver_set_pulses_per_step\n");
    NRF_LOG_INFO("Error code=%d\n", status);
#endif
    return 1;
  }
#ifdef DEBUG
  NRF_LOG_INFO("x4driver_set_pulses_per_step success\n");
#endif
  status = x4driver_set_downconversion(x4driver, 0); // Radar data as downconverted baseband IQ, not RF.
  if (status != XEP_ERROR_X4DRIVER_OK) {
#ifdef DEBUG
    NRF_LOG_INFO("Error in x4driver_set_downconversion\n");
    NRF_LOG_INFO("Error code=%d\n", status);
#endif
    return 1;
  }
#ifdef DEBUG
  NRF_LOG_INFO("x4driver_set_downconversion success\n");
#endif

  status = x4driver_set_frame_area_offset(x4driver, 0); // Given by module HW. Makes frame_area start = 0 at front of module.
  if (status != XEP_ERROR_X4DRIVER_OK) {
#ifdef DEBUG
    NRF_LOG_INFO("Error in x4driver_set_frame_area_offseto\n");
    NRF_LOG_INFO("Error code=%d\n", status);
#endif
    return 1;
  }
#ifdef DEBUG
  NRF_LOG_INFO("x4driver_set_frame_area_offset success\n");
#endif

  status = x4driver_set_frame_area(x4driver, 0, 2); // Observe from 0.5m to 4.0m.
  if (status != XEP_ERROR_X4DRIVER_OK) {
#ifdef DEBUG
    NRF_LOG_INFO("Error in x4driver_set_frame_area\n");
    NRF_LOG_INFO("Error code=%d\n", status);
#endif
    return 1;
  }
  NRF_LOG_INFO("x4driver_set_frame_area success\n");

  status = x4driver_check_configuration(x4driver);
#ifdef DEBUG
  if (status == XEP_ERROR_X4DRIVER_OK) {
    NRF_LOG_INFO("x4driver_check_configuration success\n");
  } else {
    NRF_LOG_INFO("x4driver_check_configuration unknow situcation\n");
  }
#endif // DEBUG

  /***************set fps, this will trigger data output***************/
  status = x4driver_set_fps(x4driver, 1); // Generate 5 frames per second
  if (status != XEP_ERROR_X4DRIVER_OK) {
#ifdef DEBUG
    NRF_LOG_INFO("Error in x4driver_set_fps\n");
    NRF_LOG_INFO("Error code=%d\n", status);
#endif
    return 1;
  }
#ifdef DEBUG
  NRF_LOG_INFO("x4driver_set_fps success\n");
#endif

  for (;;) {
  }
}