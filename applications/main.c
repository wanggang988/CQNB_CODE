/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <sensor.h>
#include "at.h"
#include "cm_datacom.h"
#include "drv_common.h"
#define LOG_TAG "main"
#define LOG_LVL DBG_LOG
#include "ulog.h"			//	������LOG_TAG��LOG_LVL����
#define NB_UART_NAME		"lpuart1"	/*�����豸����*/
rt_device_t nb_uart;					/*�����豸���*/
struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /*��ʼ�����ò���*/

#define RT_UPDATE_LIFETIME_THREAD_STACK_SIZE        512
#define RT_UPDATE_LIFETIME_THREAD_PRIORITY          20
/* defined the Status LED pin: PE4 */
#define STATUS_LED_PIN    GET_PIN(E, 4)
#define RAIN_IT     GET_PIN(C, 2)          //rain interrupt PC2
extern int adc_vol_sample(void);
extern int lsm6dsl_port(void);
void rain_callback(void *args);
#define ACCEL_DEVICE_NAME	"acce_lsm"
static rt_device_t accel_dev;
struct rt_sensor_data buffer;

//extern int module_error;
void nb_thread_entry(void *parameter);
void nb_wakeupthread_entry(void *parameter);
void gps_thread_entry(void *parameter);  //gps thread
void sample_thread_entry(void *parameter);
void rs485_thread_entry(void *parameter);
void failreg_thread_entry(void *parameter);
void cme50_thread_entry(void *parameter);
void heart_thread_entry(void *parameter);
void life_thread_entry(void *parameter);
void ota_thread_entry(void *args);

/* update onenet lifetime thread*/
static struct rt_thread update_lifetime_thread;
static rt_uint8_t update_lf_thread_stack[RT_UPDATE_LIFETIME_THREAD_STACK_SIZE];
//char * testchar = 
static void HextoHexStr(char* pSrc, char* pDst, unsigned int nSrcLength)
{
    int i = 0;
    const char tab[]="0123456789ABCDEF";    // 0x0-0xf���ַ����ұ�

    for (i = 0; i < nSrcLength; i++)
    {
        *pDst++ = tab[*pSrc >> 4];      // �����4λ
        *pDst++ = tab[*pSrc & 0x0f];    // �����4λ
        pSrc++;
    }

    // ����ַ����Ӹ�������
    *pDst = '\0';
}
/*update lifetime thread entry*/
void update_lifetime_thread_entry(void* parameter) 
{
    while(1)
    {
        rt_thread_delay(1000 * 60 * 60 * 6); //6h����һ����������
        
		rt_event_recv(&nb_event,NBNET,RT_EVENT_FLAG_OR,RT_WAITING_FOREVER, RT_NULL); //��������ǰ����
//        while(module_error == 1);
        at_exec_cmd(RT_NULL,"AT+MIPLUPDATE=0,86400,0");  //update lifetime
        rt_kprintf("�����������ڳɹ�\n");
    }
}
void thread_startup(void)
{
	struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /*��ʼ�����ò���*/
    rt_thread_t tid1;
    rt_thread_t tid2;
    rt_thread_t tid3;
    rt_thread_t tid4;
    rt_thread_t tid5;
    rt_thread_t tid6;
    rt_thread_t tid7;
    rt_thread_t tid8;
    rt_thread_t tid9;
//	rt_pin_mode(NB_EN_PIN, PIN_MODE_OUTPUT);
//	rt_pin_write(NB_EN_PIN, PIN_HIGH);
	/*���Ҵ����豸*/
	nb_uart = rt_device_find(NB_UART_NAME);
	/* step2���޸Ĵ������ò��� */
	config.baud_rate = BAUD_RATE_9600;        //�޸Ĳ�����Ϊ 9600
	config.data_bits = DATA_BITS_8;           //����λ 8
	config.stop_bits = STOP_BITS_1;           //ֹͣλ 1
	config.bufsz     = 512;                   //�޸Ļ����� buff size Ϊ 512
	config.parity    = PARITY_NONE;           //����żУ��λ

	/* ���ƴ����豸��ͨ�����ƽӿڴ�����������֣�����Ʋ��� */
	rt_device_control(nb_uart, RT_DEVICE_CTRL_CONFIG, &config);

	/* �򿪴����豸�����жϽ��ռ���ѯ����ģʽ�򿪴����豸 */
//	rt_device_open(nb_uart, RT_DEVICE_FLAG_DMA_RX);
	at_client_init("lpuart1",512);         //at client ��ʼ��
	/* BC35-G thread */
	tid1 = rt_thread_create("BC35-G", nb_thread_entry, RT_NULL,
                           1024 * 3, RT_MAIN_THREAD_PRIORITY, 20);		//���ȼ�10
    RT_ASSERT(tid1 != RT_NULL);
	rt_thread_startup(tid1);
	/* updata life_time_thread_init */
    rt_err_t update_lifetime_result;
    update_lifetime_result = rt_thread_init(&update_lifetime_thread,"lfthread",update_lifetime_thread_entry,
                                            RT_NULL,&update_lf_thread_stack[0],RT_UPDATE_LIFETIME_THREAD_STACK_SIZE,RT_UPDATE_LIFETIME_THREAD_PRIORITY,25); //���ȼ�20
    if(update_lifetime_result == RT_EOK) rt_thread_startup(&update_lifetime_thread);    //�����������������߳�
    /* sample thread */
    tid2 = rt_thread_create("sample", sample_thread_entry, RT_NULL,
                           1024 * 2, RT_MAIN_THREAD_PRIORITY + 4, 20);
    RT_ASSERT(tid2 != RT_NULL);
//	rt_thread_startup(tid2);
    /* gps thread */
    tid3 = rt_thread_create("gps",gps_thread_entry,RT_NULL,
                            1024 * 12,RT_MAIN_THREAD_PRIORITY + 1,20);    //GPS�߳�����10K
    RT_ASSERT(tid3 != RT_NULL);
//    rt_thread_startup(tid3);
    /* RS485 thread */
    tid4 = rt_thread_create("rs485", rs485_thread_entry, RT_NULL,
                               1024 * 3, RT_MAIN_THREAD_PRIORITY + 2, 20);     
    RT_ASSERT(tid4 != RT_NULL);
//    rt_thread_startup(tid4);
//    /* ota thread */
//    tid5 = rt_thread_create("ota", ota_thread_entry, RT_NULL,
//                               1024, RT_MAIN_THREAD_PRIORITY + 5, 20);     
//    RT_ASSERT(tid5 != RT_NULL);
//    rt_thread_startup(tid5);
    /* life thread */
    tid6 = rt_thread_create("life", life_thread_entry, RT_NULL,
                           1024 * 2, RT_MAIN_THREAD_PRIORITY + 7, 20);
    RT_ASSERT(tid6 != RT_NULL);
//	rt_thread_startup(tid6);
	/* heartbeat thread */
	tid7 = rt_thread_create("heart", heart_thread_entry, RT_NULL,
                           1024 * 2, RT_MAIN_THREAD_PRIORITY + 6, 20);
    RT_ASSERT(tid7 != RT_NULL);
//	rt_thread_startup(tid7);
	/* error handle thread */
	tid8 = rt_thread_create("failreg", failreg_thread_entry, RT_NULL,
                           1024 * 2, RT_MAIN_THREAD_PRIORITY - 1, 50); //���ȼ�10
    RT_ASSERT(tid8 != RT_NULL);
//	rt_thread_startup(tid8);
    /* module error handle thread */
	tid9 = rt_thread_create("cme50", cme50_thread_entry, RT_NULL,
                           1024 * 2, RT_MAIN_THREAD_PRIORITY - 1, 20); //���ȼ�10
    RT_ASSERT(tid9 != RT_NULL);
//	rt_thread_startup(tid9);
}
int main(void)
{
	rt_err_t res = RT_EOK;
	rt_pin_mode(STATUS_LED_PIN, PIN_MODE_OUTPUT);
	/*����Ϊ�����жϵ�����*/
    rt_pin_mode(RAIN_IT, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(RAIN_IT, PIN_IRQ_MODE_FALLING, rain_callback, RT_NULL);  //�������жϻص����������ж�PC2
    rt_pin_irq_enable(RAIN_IT, PIN_IRQ_ENABLE);                                     //ʹ���ж�
//	HAL_NVIC_SystemReset();//system reset
	thread_startup();	//�����߳�
//	lsm6dsl_port();
//	
	/*�����豸���Ʋ���IIC�豸����ȡ�豸���*/
//	accel_dev = rt_device_find(ACCEL_DEVICE_NAME);
//	rt_device_open(accel_dev,RT_DEVICE_FLAG_RDWR);
    /* set Status LED pin mode to output */
    
    while (1)
    {   
		rt_thread_mdelay(1000 * 60 * 1);
//		rt_device_read(accel_dev,0,&buffer,1); 
//		
//		LOG_D("***accel show %5d mg    %5dmg    %5d mg***\n",buffer.data.acce.x,buffer.data.acce.y,buffer.data.acce.z);
		
        rt_pin_write(STATUS_LED_PIN, PIN_HIGH);
		LOG_D("Status LED ON!");
        rt_thread_mdelay(2000);
        rt_pin_write(STATUS_LED_PIN, PIN_LOW);
		LOG_D("Status LED OFF!");
        rt_thread_mdelay(2000);
//		adc_vol_sample();
		
		
    }

    return RT_EOK;
}
uint32_t rainvalue;  //�����������
void rain_callback(void *args)
{
    rainvalue++ ;
    LOG_D("rainvalue is %d\n",rainvalue);

}