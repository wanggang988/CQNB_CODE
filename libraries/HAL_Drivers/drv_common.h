/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-7      SummerGift   first version
 */

#ifndef __DRV_COMMON_H__
#define __DRV_COMMON_H__

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

void _Error_Handler(char *s, int num);

#ifndef Error_Handler
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#endif

#define DMA_NOT_AVAILABLE ((DMA_INSTANCE_TYPE *)0xFFFFFFFFU)

#ifdef __cplusplus
}
#endif
/*GPIO ∫Í∂®“Â*/
#define LED0_PIN    GET_PIN(E, 4)       //PE4
#define V33_POWER   GET_PIN(E, 9)       //PE9 3.3V total power
#define V33_LORA    GET_PIN(B, 2)       //PB2 LoRa Power
#define V33_232     GET_PIN(E, 2)       //PE2 232 Power
#define V33_I2C     GET_PIN(E, 14)       //PE14 I2C Power
#define V33_GPS     GET_PIN(E, 12)       //PE2 GPS Power
#define RAIN_IT     GET_PIN(C, 2)          //rain interrupt PC2
#define VC5_POWER   GET_PIN(E ,8)           // 5VC POWER
#define ADC_POWER   GET_PIN(E, 10)          //ADC POWER
#define NB_EN_PIN   GET_PIN(B, 12) 			//NB POWER ON PB12
#endif
