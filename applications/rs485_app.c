#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdlib.h>
#include "cm_datacom.h"
//#include "iic.h"
#define RS485_DE_PIN    GET_PIN(C, 3)			//PC3:485�շ����ƽš�
#define deg_scale       ((float)11.111f)
#define degx_que  3
#define degy_que  7
#define mudk_que  5      //��λΪmm �ո�2���ֽ�
#define mudw_que  9      //��λΪmm λ��2���ֽ�
/*functions */
uint16_t CRC16_modbus(uint8_t *_pBuf, uint16_t _usLen);
static void parase_degree(uint8_t *buf);
void read_data(uint8_t *data,uint8_t size);
int adc_vol_sample(void);
static void parase_mud(void);
/*variables list */
uint8_t rsbuf[36];

extern uint16_t cmrain;     //�����趨��ֵ
extern uint16_t cmsample;   //�������
extern float dmqx_angle;
extern float surface_angle;     //�ر�Ƕ���ֵ float 4byte
extern float surface_dist;      //�ر�������ֵ float 4byte
extern float wall_angle;        //ǽ�ѷ�Ƕ���ֵ float 4byte
extern float wall_dist; 
extern float monitor_value[2]; //adc���ֵ��12V,������������
extern int module_error;
extern void connect_nb_net(void);
extern void disconnect_nb_net(void);
rt_device_t rsdev;   //�豸
//extern uint8_t otaflag;
//extern struct rt_timer otatimer;
extern struct cm_header cm_default_config;   //header
extern struct cm_mainmsg config_data;        //msg
//extern void nb_send_ota(char* msg, uint16_t len);
uint8_t readangle[8] = {0x01,0x03,0x00,0x01,0x00,0x04,0x15,0xc9};   //��ָ����Ǵ�����ָ��(��X��Y��Ƕ�)  ����X�����ݸ�λ��2byte����X�����ݵ�λ��2byte������Y�����ݸ�λ��2byte����Y�����ݵ�λ��2byte��
uint8_t readmud[8] = {0x01,0x03,0x00,0x00,0x00,0x0a,0xc5,0xcd};     //��λ�ƶ�ָ��
struct rt_semaphore rs_sem;  //�ź�����

rt_err_t rs_rxindicate(rt_device_t dev,rt_size_t size)      //���ݽ��ջص�����
{
    rt_sem_release(&rs_sem);        //�ͷ��ź���
	return RT_EOK;
}

void rs485_thread_entry(void *parameter)
{
	uint16_t crc;
	uint8_t ch,i = 0;
	rt_err_t err;

	rt_pin_mode(RS485_DE_PIN,PIN_MODE_OUTPUT);
	rt_pin_write(RS485_DE_PIN,PIN_LOW);
	rt_sem_init(&rs_sem,"rssem",0,RT_IPC_FLAG_FIFO);      //�ź�����ʼ�����ź�����ʼֵΪ0

	rsdev =  rt_device_find("uart4");         //�����豸
	rt_device_set_rx_indicate(rsdev,rs_rxindicate); //����4���յ����ݣ��������ݽ���ָʾ���ص�����Ϊrs_rxindicate
	err = rt_device_open(rsdev,RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX); //���豸
	if(err !=RT_EOK)
	{
	    rt_kprintf("uart4 open error\n");
	}
	
//	rx_fifo = (struct rt_serial_rx_fifo*)serial;
//	read_data(readangle,8);
//	read_data(readmud,8);    //����λ
	while(1)
	{
		if(rt_sem_take(&rs_sem,RT_WAITING_FOREVER) == RT_EOK)  //��ȡ�ź���
		{	
			rt_device_read(rsdev,0,&ch,1);	  //Ӧ�ó�����豸�ж�ȡ����
			rsbuf[i++]=ch;
			crc = (rsbuf[i-2] <<8) | rsbuf[i-1];
			if(CRC16_modbus(rsbuf,i-2) == crc)
			{
				rt_kprintf("crc16 ok = %x\n",crc);
				i=0;
				rt_kprintf("����RS485�߳�\n");

				parase_degree(rsbuf);  //�ǶȽ���
//				parase_mud();     //��λ����
//				adc_vol_sample();
                rt_kprintf("�˳�RS485�߳�\n");
			}
			else if(i>36)
			{
				i=0;
			}
			
		}
//        rt_pm_request(PM_SLEEP_MODE_DEEP);    //����������˯��ģʽ
	}
}


float deg_x,deg_y,deg_z;   //������б��Ǵ��������ֵ
float degx_last, degy_last;
static void parase_degree(uint8_t *buf)
{
	int high,low;
	float f;
    uint16_t len;
    cm_header_t ptr;
    ptr = & cm_default_config;
	high = ((buf[degx_que] <<8) | buf[degx_que+1]) - 10000;
	low = ((buf[degx_que+2] <<8) | buf[degx_que+3]) - 10000;
	f =  (float)low/10000;
	deg_x = high + f;
	rt_kprintf("deg x =%d\n",high);
	
	high = ((buf[degy_que] <<8) | buf[degy_que+1]) - 10000;
	low = ((buf[degy_que+2] <<8) | buf[degy_que+3]) - 10000;
	f = (float)low/10000;
	deg_y = high + f;
    rt_kprintf("deg y =%d\n",high);
    if(labs(deg_x - degx_last) > dmqx_angle || labs(deg_y - degy_last) > dmqx_angle)        // dmqx_angle ������б�Ƕ���ֵ ������ֵ���ϱ�
    {
        refreshdata();
        config_data.cmd = cm_modulestatus_up;

        ptr->dev_type = CM_ANGLE_DEV;     //
        ptr->dev_id = CM_DMQX_ID;        

        len = cm_encode(ptr,&config_data);
//        while(module_error == 1);
        nb_send((char*)cm_senddata,len);      //�ϱ���Ҫ��Ϣ
    }
	degx_last = deg_x;
    degy_last = deg_y;
}

float mudhigh_last,mudhigh;
extern float mudthreshold;
extern void refreshdata(void);
uint16_t nullhigh,liqhigh;
static void parase_mud()
{
	uint16_t l;
	cm_header_t  ptr;
	ptr = & cm_default_config;   //header
	nullhigh = (rsbuf[mudk_que] <<8) | rsbuf[mudk_que+1];        //��λmm
	liqhigh = (rsbuf[mudw_que] <<8) | rsbuf[mudw_que+1];         //λ�ߵ�λmm
	rt_kprintf("liqhigh = %d mm\n",liqhigh);
	if(liqhigh > mudthreshold)
	{
		refreshdata();
		config_data.cmd = cm_modulestatus_up;
        ptr->dev_type = CM_MUD_DEV;     //��Ϊ��λ
        ptr->dev_id = CM_MUD_ID;
		l = cm_encode(ptr,&config_data);
//        while(module_error == 1);
		nb_send((char*)cm_senddata,l);      //�ϱ���Ҫ��Ϣ
	}
	
}

int16_t acc_init[3],acc_last[3] = {0};//��ʼ��ƽֵ̨
static void parase_accdegree(int16_t* newdata)
{
	uint8_t i,l;
	int p[3];                                            
	float degree[3],relatdeg[3];
	cm_header_t  ptr;
	ptr = &cm_default_config;
	for(i=0;i<3;i++)
	{
		degree[i] = (newdata[i] - acc_init[i])/deg_scale;      //���ԽǶ�
		relatdeg[i] = (newdata[i] - acc_last[i])/deg_scale;      //��ԽǶ�
	}
	deg_x = degree[0];
	deg_y = degree[1];
	deg_z = degree[2];
	#if USING_DBLH //�ر��ѷ� 
	if(labs(relatdeg[0]) > surface_angle || labs(relatdeg[1]) > surface_angle || labs(relatdeg[2]) > surface_angle)
	{
//		refreshdata();
	    config_data.cmd = cm_modulestatus_up;
		l = cm_encode(ptr,&config_data);
//        while(module_error == 1);
		nb_send((char*)cm_senddata,l);      //�ϱ���Ҫ��Ϣ
		rt_kprintf("��ǳ�����ֵ\r\n");
	}
    #endif
    #if USING_QLH //ǽ�ѷ� 
	if(labs(relatdeg[0]) > wall_angle || labs(relatdeg[1]) > wall_angle || labs(relatdeg[2]) > wall_angle)
	{
		refreshdata();
	    config_data.cmd = cm_modulestatus_up;
		l = cm_encode(ptr,&config_data);
		nb_send((char*)cm_senddata,l);      //�ϱ���Ҫ��Ϣ
		rt_kprintf("��ǳ�����ֵ\r\n");
	}
    #endif
	for(i=0;i<3;i++)
	{
		acc_last[i] = newdata[i];
	}

}

void read_data(uint8_t *data,uint8_t size)
{
//	rt_pin_write(35,PIN_HIGH);   //����
//	rt_device_write(rsdev,0,data,size);
	rt_pin_write(RS485_DE_PIN,PIN_LOW);     //����
}
/* �����߳���� */

void sample_thread_entry(void *parameter)        //����Ƶ��
{
    cm_header_t  ptr;
    float lastwire;
	uint32_t last;
	uint16_t l;
    int16_t accdata[3];
	ptr = &cm_default_config;
    rt_thread_delay(1000 * 15);
    rt_thread_delay(1000 * 3);
//	LSM6DSL_AccInit(0x30 | (0x44<<8));   //iic��ʼ��
	rt_thread_delay(1000 * 60 * 2);     //delay 4min
    rt_event_recv(&nb_event,NBNET,RT_EVENT_FLAG_OR ,RT_WAITING_FOREVER, RT_NULL);
    
	while(1)
	{
        rt_thread_delay(cmsample); //
		rt_kprintf("����ɼ��߳�\n");
//        accreadxyz(accdata);
//		parase_accdegree(accdata); //���ٶȽ�������
        adc_vol_sample();
		read_data(readangle,8);
//		read_data(readmud,8);
//        rt_kprintf("rain value = %d\r\n",rainvalue);
//        if((rainvalue - last) * 0.1 > cmrain)      //������ֵ������ֵ
//        {
//            refreshdata();  
//            config_data.cmd = cm_modulestatus_up;
//            ptr->dev_type = CM_RAIN_DEV;     //��Ϊ������һ��Ҫ�ģ�������ֵ���ϱ�
//            ptr->dev_id = CM_RAIN_ID;
//            l = cm_encode(ptr,&config_data);
//            nb_send((char*)cm_senddata,l);      //�ϱ���Ҫ��Ϣ(������
//        }
//        last = rainvalue;
        
        #if USING_DBLH
        if((monitor_value[1] - lastwire) > surface_dist)      //����ֵ
		{
			refreshdata();
			config_data.cmd = cm_modulestatus_up;
            ptr->dev_type = CM_EARTH_DEV;     //��Ϊ�ر��ѷ죬һ��Ҫ�ģ�������ֵ���ϱ�
            ptr->dev_id = CM_DBLF_ID;
			l = cm_encode(ptr,&config_data);
			nb_send((char*)cm_senddata,l);      //�ϱ���Ҫ��Ϣ
            rt_kprintf("�ر����߳�����ֵ\r\n");
		}
        lastwire = monitor_value[1];
        #endif
        #if USING_QLH
        if((monitor_value[1] - lastwire) > wall_dist)      //����
		{
			refreshdata();
			config_data.cmd = cm_modulestatus_up;
            ptr->dev_type = CM_WALL_DEV;     //��Ϊǽ�ѷ죬һ��Ҫ�ģ�������ֵ���ϱ�
            ptr->dev_id = CM_QLF_ID;
			l = cm_encode(ptr,&config_data);
			nb_send((char*)cm_senddata,l);      //�ϱ���Ҫ��Ϣ
			rt_kprintf("ǽ���߳�����ֵ\r\n");
		}
        lastwire = monitor_value[1];
        #endif
        rt_kprintf("�˳��ɼ��߳�\n");
//        rt_pm_request(PM_SLEEP_MODE_DEEP);    //����������˯��ģʽ
        rt_thread_delay(1000 * 20);
    }	
}