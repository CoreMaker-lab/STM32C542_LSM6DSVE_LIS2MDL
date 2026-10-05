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
#define BOOT_TIME 10 // ms

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);

static volatile uint8_t thread_wake = 0;

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len) {
  if (HAL_I2C_MASTER_MemWrite((hal_i2c_handle_t *)handle, LSM6DSV_I2C_ADD_L,
                              reg, HAL_I2C_MEM_ADDR_8BIT, bufp, len, 1000) != HAL_OK) {
    return -1;
  }
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len) {
  if (HAL_I2C_MASTER_MemRead((hal_i2c_handle_t *)handle, LSM6DSV_I2C_ADD_L,
                             reg, HAL_I2C_MEM_ADDR_8BIT, bufp, len, 1000) != HAL_OK) {
    return -1;
  }
  return 0;
}

static void platform_delay(uint32_t ms) {
  HAL_Delay(ms);
}

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

    stmdev_ctx_t dev_ctx;
    lsm6dsv_tap_detection_t    tap     = {0};  /* 锟街讹拷: tap_x_en/tap_y_en/tap_z_en */
    lsm6dsv_tap_thresholds_t   tap_ths = {0};  /* 锟街讹拷: x/y/z */
    lsm6dsv_tap_time_windows_t tap_win = {0};  /* 锟街讹拷: shock/quiet/tap_gap */
    lsm6dsv_pin_int_route_t    pin_int = {0};  /* 锟街讹拷: single_tap/double_tap/drdy_xl */
    lsm6dsv_interrupt_mode_t irq = {0};
    lsm6dsv_if_cfg_t if_cfg = {0};

	  /* Initialize mems driver interface */
	  dev_ctx.write_reg = platform_write;
	  dev_ctx.read_reg = platform_read;
	  dev_ctx.mdelay = platform_delay;
	  dev_ctx.handle = mx_i2c1_i2c_gethandle();

	  /* Init test platform */
      // platform_init(dev_ctx.handle);

	  /* Wait sensor boot time */
	  platform_delay(BOOT_TIME);

	  /* Check device ID */
	  lsm6dsv_device_id_get(&dev_ctx, &whoamI);
	  printf("LSM6DSV_ID=0x%x,id=0x%x\n",LSM6DSV_ID,whoamI);
	  if (whoamI != LSM6DSV_ID)
	    while (1);

	  /* Restore default configuration */
	  lsm6dsv_reset_set(&dev_ctx, LSM6DSV_RESTORE_CTRL_REGS);

	  /* Enable Block Data Update */
	  lsm6dsv_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

	  /* Set Output Data Rate.
	   * Selected data rate have to be equal or greater with respect
	   * with MLC data rate.
	   */
	  lsm6dsv_xl_data_rate_set(&dev_ctx, LSM6DSV_ODR_AT_480Hz);
	  lsm6dsv_gy_data_rate_set(&dev_ctx, LSM6DSV_ODR_AT_15Hz);

	  /* Set full scale */
	  lsm6dsv_xl_full_scale_set(&dev_ctx, LSM6DSV_8g);
	  lsm6dsv_gy_full_scale_set(&dev_ctx, LSM6DSV_2000dps);

    /* INT1/INT2: open-drain + active low (matches external pull-up + PB0 falling edge) */
    lsm6dsv_read_reg(&dev_ctx, LSM6DSV_IF_CFG, (uint8_t *)&if_cfg, 1);
    if_cfg.pp_od     = 1;
    if_cfg.h_lactive = 1;
    lsm6dsv_write_reg(&dev_ctx, LSM6DSV_IF_CFG, (uint8_t *)&if_cfg, 1);

      /* Enable Z-axis Tap detection */
      tap.tap_x_en = PROPERTY_DISABLE;
      tap.tap_y_en = PROPERTY_DISABLE;
      tap.tap_z_en = PROPERTY_ENABLE;
      lsm6dsv_tap_detection_set(&dev_ctx, tap);

      /* Set Tap threshold */
      tap_ths.x = 0;
      tap_ths.y = 0;
      tap_ths.z = 3;
      lsm6dsv_tap_thresholds_set(&dev_ctx, tap_ths);

      /* Set Tap time windows */
      tap_win.shock = 3;
      tap_win.quiet = 3;
      tap_win.tap_gap = 7;
      lsm6dsv_tap_time_windows_set(&dev_ctx, tap_win);

      /* Enable Single Tap and Double Tap */
      lsm6dsv_tap_mode_set(&dev_ctx,
                               LSM6DSV_BOTH_SINGLE_DOUBLE);

      /* Route Single Tap and Double Tap to INT1 */
      pin_int.single_tap = PROPERTY_ENABLE;
      pin_int.double_tap = PROPERTY_ENABLE;

     /* enable interrupt on High-G XL (sensor at highest frequency) */
	  pin_int.drdy_xl = PROPERTY_DISABLE;
      irq.enable = PROPERTY_ENABLE;
      irq.lir    = 0;
      lsm6dsv_interrupt_enable_set(&dev_ctx, irq);
	  lsm6dsv_pin_int1_route_set(&dev_ctx, &pin_int);


	  while (1) {

		if (thread_wake)
		{
		  lsm6dsv_all_sources_t status = {0};

		  thread_wake = 0;

		  /* Read interrupt source */
		  lsm6dsv_all_sources_get(&dev_ctx, &status);

		  if (status.double_tap)
		  {
			printf("Double Tap\r\n");
		  }
		  else if (status.single_tap)
		  {
			printf("Single Tap\r\n");
		  }
	    }
	  }
  }
} /* end main */


