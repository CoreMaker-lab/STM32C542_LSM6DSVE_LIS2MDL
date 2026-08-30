/**
  ******************************************************************************
  * file           : main.c
  * brief          : Main program body
  *                  Calls target system initialization then loop in main.
  ******************************************************************************
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions prototype -----------------------------------------------*/

#include "mx_usart1.h"
#include <stdio.h>
#include <string.h>

#include "lsm6dsv_reg.h"

int _write(int file, char *ptr, int len)
{
    hal_uart_handle_t *huart1 = mx_usart1_uart_gethandle();

    if (huart1 != NULL)
    {
        HAL_UART_Transmit(huart1, ptr, len, 1000);
    }

    return len;
}

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms
#define    CNT_FOR_OUTPUT       100

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float_t acceleration_mg[3];
static float_t angular_rate_mdps[3];
static float_t temperature_degC;
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

static lsm6dsv_filt_settling_mask_t filt_settling_mask;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void *handle);

static   stmdev_ctx_t dev_ctx;
static   uint8_t lg_xl_data_valid = 0;
static   uint8_t hg_xl_data_valid = 0;
static   uint8_t gyro_data_valid = 0;
static   uint8_t temp_data_valid = 0;
static   volatile uint8_t thread_wake = 0;

void HAL_EXTI_TriggerCallback(hal_exti_handle_t *hexti,
                              hal_exti_trigger_t trigger)
{
    if (HAL_EXTI_GetInstance(hexti) == HAL_EXTI_GPIO_0)
    {
        thread_wake = 1;
    }
}
/**
  * brief:  The application entry point.
  * retval: none but we specify int to comply with C99 standard
  */
int main(void)
{
  /** System Init: this code placed in targets folder initializes your system.
    * It calls the initialization (and sets the initial configuration) of the peripherals.
    * You can use STM32CubeMX to generate and call this code or not in this project.
    * It also contains the HAL initialization and the initial clock configuration.
    */
  if (mx_system_init() != SYSTEM_OK)
  {
    return (-1);
  }
  else
  {
    /*
      * You can start your application code here
      */

	  printf("HELLO\n");
	  HAL_GPIO_WritePin(CS1_PORT, CS1_PIN, HAL_GPIO_PIN_SET);
	  HAL_GPIO_WritePin(SA0_PORT, SA0_PIN, HAL_GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(CS2_PORT, CS2_PIN, HAL_GPIO_PIN_SET);

	  lsm6dsv_pin_int_route_t pin_int = { 0 };
	  stmdev_ctx_t dev_ctx;
	  double_t lowg_xl_sum[3], hg_xl_sum[3], gyro_sum[3], temp_sum;
	  uint16_t lowg_xl_cnt = 0, hg_xl_cnt = 0, gyro_cnt = 0, temp_cnt = 0;

	  /* Initialize mems driver interface */
	  dev_ctx.write_reg = platform_write;
	  dev_ctx.read_reg = platform_read;
	  dev_ctx.mdelay = platform_delay;
	  dev_ctx.handle = mx_i2c1_i2c_gethandle();

	  /* Init test platform */
//	  platform_init();

	  /* Wait sensor boot time */
	  platform_delay(BOOT_TIME);

	  /* Check device ID */
	  lsm6dsv_device_id_get(&dev_ctx, &whoamI);
	  printf("LSM6DSV_ID=0x%x,id=0x%x\n",LSM6DSV_ID,whoamI);
	  if (whoamI != LSM6DSV_ID)
	    while (1);

	  /* Perform device power-on-reset */
	  lsm6dsv_sw_por(&dev_ctx);

	  /* Enable Block Data Update */
	  lsm6dsv_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

	  /* Set Output Data Rate.
	   * Selected data rate have to be equal or greater with respect
	   * with MLC data rate.
	   */
	  lsm6dsv_xl_data_rate_set(&dev_ctx, LSM6DSV_ODR_AT_15Hz);
	  lsm6dsv_gy_data_rate_set(&dev_ctx, LSM6DSV_ODR_AT_15Hz);
	  /* Set full scale */
	  lsm6dsv_xl_full_scale_set(&dev_ctx, LSM6DSV_2g);
	  lsm6dsv_gy_full_scale_set(&dev_ctx, LSM6DSV_2000dps);

	  /* Configure filtering chain */
	  filt_settling_mask.drdy = PROPERTY_ENABLE;
	  filt_settling_mask.irq_xl = PROPERTY_ENABLE;
	  filt_settling_mask.irq_g = PROPERTY_ENABLE;
	  lsm6dsv_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
	  lsm6dsv_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
	  lsm6dsv_filt_gy_lp1_bandwidth_set(&dev_ctx, LSM6DSV_GY_ULTRA_LIGHT);
	  lsm6dsv_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
	  lsm6dsv_filt_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSV_XL_STRONG);

	  lowg_xl_sum[0] = lowg_xl_sum[1] = lowg_xl_sum[2] = 0.0;
	  hg_xl_sum[0] = hg_xl_sum[1] = hg_xl_sum[2] = 0.0;
	  gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0.0;
	  temp_sum = 0.0;

	  lsm6dsv_if_cfg_t if_cfg = {0};

	  lsm6dsv_read_reg(&dev_ctx,
	                       LSM6DSV_IF_CFG,
	                       (uint8_t *)&if_cfg,
	                       1);

	  /* INT1/INT2: open-drain + active low */
	  if_cfg.pp_od = 1;
	  if_cfg.h_lactive = 1;

	  lsm6dsv_write_reg(&dev_ctx,
	                        LSM6DSV_IF_CFG,
	                        (uint8_t *)&if_cfg,
	                        1);
	  /* enable interrupt on High-G XL (sensor at highest frequency) */
	  pin_int.drdy_xl = PROPERTY_ENABLE;
	  lsm6dsv_pin_int1_route_set(&dev_ctx, &pin_int);
	  //lsm6dsv_pin_int2_route_set(&dev_ctx, &pin_int);

    while (1) {
        if (thread_wake) {
        	lsm6dsv_all_sources_t all_sources;
            /* Read output only if new xl value is available */
            lsm6dsv_all_sources_get(&dev_ctx, &all_sources);

            if (all_sources.drdy_xl) {
              /* Read acceleration field data */
              memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
              lsm6dsv_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
              acceleration_mg[0] =
                lsm6dsv_from_fs2_to_mg(data_raw_acceleration[0]);
              acceleration_mg[1] =
                lsm6dsv_from_fs2_to_mg(data_raw_acceleration[1]);
              acceleration_mg[2] =
                lsm6dsv_from_fs2_to_mg(data_raw_acceleration[2]);
              printf("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                      acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
            }

            if (all_sources.drdy_gy) {
              /* Read angular rate field data */
              memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
              lsm6dsv_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
              angular_rate_mdps[0] =
                lsm6dsv_from_fs2000_to_mdps(data_raw_angular_rate[0]);
              angular_rate_mdps[1] =
                lsm6dsv_from_fs2000_to_mdps(data_raw_angular_rate[1]);
              angular_rate_mdps[2] =
                lsm6dsv_from_fs2000_to_mdps(data_raw_angular_rate[2]);
              printf("Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
                      angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
            }

            if (all_sources.drdy_temp) {
              /* Read temperature data */
              memset(&data_raw_temperature, 0x00, sizeof(int16_t));
              lsm6dsv_temperature_raw_get(&dev_ctx, &data_raw_temperature);
              temperature_degC = lsm6dsv_from_lsb_to_celsius(
                                   data_raw_temperature);
              printf("Temperature [degC]:%6.2f\r\n", temperature_degC);
            }






        }



    }
  }
} /* end main */

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
    hal_i2c_handle_t *hi2c = (hal_i2c_handle_t *)handle;

    if (HAL_I2C_MASTER_MemWrite(hi2c,
    		LSM6DSV_I2C_ADD_L,
                                reg,
                                HAL_I2C_MEM_ADDR_8BIT,
                                bufp,
                                len,
                                1000) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{

    hal_i2c_handle_t *hi2c = (hal_i2c_handle_t *)handle;

    if (HAL_I2C_MASTER_MemRead(hi2c,
    		LSM6DSV_I2C_ADD_L,
                               reg,
                               HAL_I2C_MEM_ADDR_8BIT,
                               bufp,
                               len,
                               1000) != HAL_OK)
    {
        return -1;
    }

    return 0;
}



/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{

  HAL_Delay(ms);
}


