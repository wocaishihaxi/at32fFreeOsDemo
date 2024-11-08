/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"
#include "FreeRTOS.h"
#include "task.h"

/** @addtogroup UTILITIES_examples
  * @{
  */

/** @addtogroup FreeRTOS_demo
  * @{
  */
TaskHandle_t led2_handler;
TaskHandle_t led3_handler;

/* led2 task */
void led2_task_function(void *pvParameters);
/* led3 task */
void led3_task_function(void *pvParameters);


void Uart1Gpio_init(void)
{
	gpio_init_type gpio_init_struct;
	/* 使能 GPIOB 的时钟 */ 
	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);  
	crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE); 
	 
	/* 将 USART1的TX和RX重映设到PB8，PB9，复用映射定义参见表6 */ 
	gpio_pin_remap_config(USART1_MUX, TRUE); 
	 
	/* 将 USART5_Tx 和 USART5_Rx 配置为复用功能 */ 
	gpio_init_struct.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7; 
	gpio_init_struct.gpio_mode = GPIO_MODE_MUX; 
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE; 
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL; 
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER; 
	gpio_init(GPIOB, &gpio_init_struct);
}

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  system_clock_config();

  /* init led2 and led3 */
  at32_led_init(LED2);
  at32_led_init(LED3);
	
	Uart1Gpio_init();

  /* init usart1 */
  uart_print_init(9600);

	printf("Hello.\n");	
	
  /* enter critical */
  taskENTER_CRITICAL();

  /* create led2 task */
  if(xTaskCreate((TaskFunction_t )led2_task_function,
                 (const char*    )"LED2_task",
                 (uint16_t       )512,
                 (void*          )NULL,
                 (UBaseType_t    )2,
                 (TaskHandle_t*  )&led2_handler) != pdPASS)
  {
    printf("LED2 task could not be created as there was insufficient heap memory remaining.\r\n");
  }
  else
  {
    printf("LED2 task was created successfully.\r\n");
  }
  /* create led3 task */
  if(xTaskCreate((TaskFunction_t )led3_task_function,
                 (const char*    )"LED3_task",
                 (uint16_t       )512,
                 (void*          )NULL,
                 (UBaseType_t    )2,
                 (TaskHandle_t*  )&led3_handler) != pdPASS)
  {
    printf("LED3 task could not be created as there was insufficient heap memory remaining.\r\n");
  }
  else
  {
    printf("LED3 task was created successfully.\r\n");
  }

  /* exit critical */
  taskEXIT_CRITICAL();

  /* start scheduler */
  vTaskStartScheduler();
}

/* led2 task function */
void led2_task_function(void *pvParameters)
{
  while(1)
  {
    at32_led_toggle(LED2);
	  printf("LED2 toggle.\r\n");
    vTaskDelay(1000);
  }
}

/* led3 task function */
void led3_task_function(void *pvParameters)
{
  while(1)
  {
    at32_led_toggle(LED3);
	  printf("LED3 toggle.\r\n");
    vTaskDelay(500);
  }
}

/**
  * @}
  */

/**
  * @}
  */
