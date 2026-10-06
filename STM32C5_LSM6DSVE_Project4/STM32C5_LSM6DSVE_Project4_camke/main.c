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
#define    BOOT_TIME            10 //ms
/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;
/* Extern variables ----------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void platform_delay(uint32_t ms);
static stmdev_ctx_t dev_ctx;
static volatile uint8_t thread_wake = 0;
/* STM32 HAL expects the 8-bit I2C address.
 * SA0 = 0 -> 0xD5 (see LSM6DSV_I2C_ADD_L). */
#define LSM6DSV_I2C_ADD  LSM6DSV_I2C_ADD_L

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  if (HAL_I2C_MASTER_MemWrite((hal_i2c_handle_t *)handle, LSM6DSV_I2C_ADD, reg,
                              HAL_I2C_MEM_ADDR_8BIT, bufp, len, 1000) != HAL_OK)
    return -1;
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  /* NOTE: the 0x80 read bit is only required on the SPI interface.
   * On I2C the register address must be sent as-is. */
  if (HAL_I2C_MASTER_MemRead((hal_i2c_handle_t *)handle, LSM6DSV_I2C_ADD, reg,
                             HAL_I2C_MEM_ADDR_8BIT, bufp, len, 1000) != HAL_OK)
    return -1;
  return 0;
}

static void platform_delay(uint32_t ms) {
  HAL_Delay(ms);
}

/* EXTI line (INT1 on PB0) trigger callback: wake up the main loop. */
void HAL_EXTI_TriggerCallback(hal_exti_handle_t *hexti, hal_exti_trigger_t trigger)
{
  (void)hexti;
  (void)trigger;
  thread_wake = 1;
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

      lsm6dsv_stpcnt_mode_t pedometer = {0};
      lsm6dsv_emb_pin_int_route_t pin_int = {0};
      lsm6dsv_if_cfg_t if_cfg = {0};

      uint16_t step_count = 0;

      /* Initialize mems driver interface */
      dev_ctx.write_reg = platform_write;
      dev_ctx.read_reg = platform_read;
      dev_ctx.mdelay = platform_delay;
      dev_ctx.handle = mx_i2c1_i2c_gethandle();

      /* Wait sensor boot time */
      platform_delay(BOOT_TIME);

      /* Check device ID */
      lsm6dsv_device_id_get(&dev_ctx, &whoamI);

      printf("LSM6DSV_ID=0x%x,id=0x%x\n",
             LSM6DSV_ID, whoamI);

      if (whoamI != LSM6DSV_ID)
        while (1);

      /* Perform device power-on-reset */
      lsm6dsv_reset_set(&dev_ctx,LSM6DSV_GLOBAL_RST);
      platform_delay(10);

      /* Restore control registers */
      lsm6dsv_reset_set(&dev_ctx,LSM6DSV_RESTORE_CTRL_REGS);
      platform_delay(10);

      /* Enable Block Data Update */
      lsm6dsv_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

      /*
       * Pedometer works internally at 30 Hz.
       * Accelerometer ODR must be >= 30 Hz.
       */
      lsm6dsv_xl_data_rate_set(&dev_ctx,LSM6DSV_ODR_AT_30Hz);
      lsm6dsv_xl_mode_set(&dev_ctx,LSM6DSV_XL_HIGH_PERFORMANCE_MD);

      /* Set accelerometer full scale */
      lsm6dsv_xl_full_scale_set(&dev_ctx,LSM6DSV_2g);

      /* INT1/INT2: open-drain + active low */
      lsm6dsv_read_reg(&dev_ctx,LSM6DSV_IF_CFG,(uint8_t *)&if_cfg,1);

      if_cfg.pp_od = 1;
      if_cfg.h_lactive = 1;

      lsm6dsv_write_reg(&dev_ctx,LSM6DSV_IF_CFG,(uint8_t *)&if_cfg,1);

      /* Enable pedometer and step counter */
      pedometer.step_counter_enable = PROPERTY_ENABLE;

      lsm6dsv_stpcnt_mode_set(&dev_ctx,pedometer);

      /*
       * Set pedometer debounce.
       *
       * Default value = 10 steps.
       * Set to 3 steps for easier demonstration.
       */
      lsm6dsv_stpcnt_debounce_set(&dev_ctx,3);

      /* Reset step counter */
      lsm6dsv_stpcnt_rst_step_set(&dev_ctx,PROPERTY_ENABLE);

      /* Route Step Detector event to INT1 */
      pin_int.step_det = PROPERTY_ENABLE;
      lsm6dsv_emb_pin_int1_route_set(&dev_ctx,&pin_int);

      printf("Pedometer start...\r\n");

      while (1) {

          if (thread_wake)
          {
            lsm6dsv_all_sources_t status = {0};

            thread_wake = 0;

            /* Read interrupt source */
            lsm6dsv_all_sources_get(&dev_ctx,
                                    &status);

            /* Check Step Detector event */
            if (status.step_detector)
            {
              /* Read Step Counter */
              lsm6dsv_stpcnt_steps_get(&dev_ctx,
                                       &step_count);

              printf("Step detected, Steps = %d\r\n",
                     step_count);
            }
          }
      }
  }
} /* end main */

