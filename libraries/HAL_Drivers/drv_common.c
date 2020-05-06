/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-7      SummerGift   first version
 */

#include "drv_common.h"
/*GPIO 宏定义*/
//#define LED0_PIN    GET_PIN(E, 4)       //PE4
//#define V33_POWER   GET_PIN(E, 9)       //PE9 3.3V total power
//#define V33_LORA    GET_PIN(B, 2)       //PB2 LoRa Power
//#define V33_232     GET_PIN(E, 2)       //PE2 232 Power
//#define V33_I2C     GET_PIN(E, 14)       //PE14 I2C Power
//#define V33_GPS     GET_PIN(E, 12)       //PE2 GPS Power
//#define RAIN_IT     GET_PIN(C, 2)          //rain interrupt PC2
//#define VC5_POWER   GET_PIN(E ,8)           // 5VC POWER
//#define ADC_POWER   GET_PIN(E, 10)          //ADC POWER
void close_all_power(void);

#ifdef RT_USING_SERIAL
#include "drv_usart.h"
#endif

#ifdef RT_USING_FINSH
#include <finsh.h>
static void reboot(uint8_t argc, char **argv)
{
    rt_hw_cpu_reset();
}
FINSH_FUNCTION_EXPORT_ALIAS(reboot, __cmd_reboot, Reboot System);
#endif /* RT_USING_FINSH */

/* SysTick configuration */
void rt_hw_systick_init(void)
{
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / RT_TICK_PER_SECOND);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
 * This is the timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_IncTick();
    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

uint32_t HAL_GetTick(void)
{
    return rt_tick_get() * 1000 / RT_TICK_PER_SECOND;
}

void HAL_SuspendTick(void)
{
}

void HAL_ResumeTick(void)
{
}

void HAL_Delay(__IO uint32_t Delay)
{
}

/* re-implement tick interface for STM32 HAL */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    /* Return function status */
    return HAL_OK;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char *s, int num)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler */
}

/**
 * This function will delay for some us.
 *
 * @param us the delay time of us
 */
void rt_hw_us_delay(rt_uint32_t us)
{
    rt_uint32_t start, now, delta, reload, us_tick;
    start = SysTick->VAL;
    reload = SysTick->LOAD;
    us_tick = SystemCoreClock / 1000000UL;
    do {
        now = SysTick->VAL;
        delta = start > now ? start - now : reload + start - now;
    } while(delta < us_tick * us);
}

/**
 * This function will initial STM32 board.
 */
RT_WEAK void rt_hw_board_init()
{
#ifdef SCB_EnableICache
    /* Enable I-Cache---------------------------------------------------------*/
    SCB_EnableICache();
#endif

#ifdef SCB_EnableDCache
    /* Enable D-Cache---------------------------------------------------------*/
    SCB_EnableDCache();
#endif

    /* HAL_Init() function is called at the beginning of the program */
    HAL_Init();

    /* System clock initialization */
    SystemClock_Config();
    rt_hw_systick_init();

    /* Heap initialization */
#if defined(RT_USING_HEAP)
    rt_system_heap_init((void *)HEAP_BEGIN, (void *)HEAP_END);
#endif

    /* Pin driver initialization is open by default */
#ifdef RT_USING_PIN
    rt_hw_pin_init();
#endif

    /* USART driver initialization is open by default */
#ifdef RT_USING_SERIAL
    rt_hw_usart_init();
#endif

    /* Set the shell console output device */
#ifdef RT_USING_CONSOLE
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

    /* Board underlying hardware initialization */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif


    rt_pin_mode(V33_POWER, PIN_MODE_OUTPUT);
    rt_pin_mode(V33_LORA, PIN_MODE_OUTPUT);
    rt_pin_mode(V33_232, PIN_MODE_OUTPUT);
    rt_pin_mode(V33_GPS, PIN_MODE_OUTPUT);
    rt_pin_mode(VC5_POWER, PIN_MODE_OUTPUT);
    rt_pin_mode(ADC_POWER, PIN_MODE_OUTPUT);
	rt_pin_mode(V33_I2C, PIN_MODE_OUTPUT);
	rt_pin_mode(NB_EN_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(V33_TF, PIN_MODE_OUTPUT);
	rt_pin_mode(V33_485, PIN_MODE_OUTPUT);
	rt_pin_mode(V12_ANGLE, PIN_MODE_OUTPUT);
	
	rt_pin_write(VC5_POWER, PIN_HIGH);
    rt_pin_write(ADC_POWER, PIN_HIGH);
    rt_pin_write(V33_POWER, PIN_HIGH);
	
	rt_pin_write(V33_I2C, PIN_HIGH);          /*开启I2C电源*/
    rt_pin_write(V33_LORA, PIN_HIGH);
    rt_pin_write(V33_232, PIN_HIGH);
    rt_pin_write(V33_GPS, PIN_HIGH);
	rt_pin_write(V33_485, PIN_HIGH);
	rt_pin_write(V33_TF, PIN_HIGH);       
	rt_pin_write(V12_ANGLE,PIN_HIGH);       //V12_ANGLE
	
}

void entry_low_power(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Enable GPIOs clock */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE(); //PC2雨量中断
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	__HAL_RCC_GPIOH_CLK_ENABLE(); 
	/* Set all GPIO in analog state to reduce power consumption,                */
	/*   except GPIOC to keep user button interrupt enabled                     */
	/* Note: Debug using ST-Link is not possible during the execution of this   */
	/*       example because communication between ST-link and the device       */
	/*       under test is done through UART. All GPIO pins are disabled (set   */
	/*       to analog input mode) including  UART I/O pins.                    */
	GPIO_InitStructure.Pin = GPIO_PIN_All;
	GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); 
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure); 
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);

	HAL_GPIO_Init(GPIOH, &GPIO_InitStructure); 
	/* Disable GPIOs clock */
	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOD_CLK_DISABLE();
	__HAL_RCC_GPIOE_CLK_DISABLE();
	__HAL_RCC_GPIOF_CLK_DISABLE();
	__HAL_RCC_GPIOH_CLK_DISABLE();
	
//	rt_pin_write(VC5_POWER, PIN_LOW);
//    rt_pin_write(ADC_POWER, PIN_LOW);
//    rt_pin_write(V33_POWER, PIN_LOW);
//	
//	rt_pin_write(V33_I2C, PIN_LOW);
//    rt_pin_write(V33_LORA, PIN_LOW);
//    rt_pin_write(V33_232, PIN_LOW);
//    rt_pin_write(V33_GPS, PIN_LOW);
//	rt_pin_write(V33_485, PIN_LOW);
//	rt_pin_write(V33_TF, PIN_LOW);       
//	rt_pin_write(V12_ANGLE,PIN_LOW);      
}
