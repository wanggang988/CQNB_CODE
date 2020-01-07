#include <rtthread.h>
#include "nmea.h"
#include "drv_common.h"
#include "devtype.h"
extern struct rt_event nb_event;
struct rt_semaphore gps_sem;  //信号量集
uint8_t gpsbuf[1024] = {0};
float deg_lat,deg_lon; //经纬度
rt_err_t gps_rxindicate(rt_device_t dev,rt_size_t size)
{
	rt_sem_release(&gps_sem);     //释放信号量
	return RT_EOK;
	
}


void gps_thread_entry(void *parameter)
{
	struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /*初始化配置参数*/
	nmeaINFO info;
	double lat,lon;
	rt_err_t err;
	rt_sem_init(&gps_sem,"gpssem",0,RT_IPC_FLAG_FIFO);   //信号量初始化（静态信号量对象）信号量初始值为0
	
	rt_device_t dev;
	
	struct rt_serial_device *serial;
	struct rt_serial_rx_fifo* rx_fifo;
	
	dev = rt_device_find("uart2");
	/* step2：修改串口配置参数 */
	config.baud_rate = BAUD_RATE_9600;        //修改波特率为 9600
	config.data_bits = DATA_BITS_8;           //数据位 8
	config.stop_bits = STOP_BITS_1;           //停止位 1
	config.bufsz     = 1024;                   //修改缓冲区 buff size 为 512
	config.parity    = PARITY_NONE;           //无奇偶校验位

	/* 控制串口设备。通过控制接口传入命令控制字，与控制参数 */
	rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &config);
	serial = (struct rt_serial_device *)dev;
	rt_device_set_rx_indicate(dev,gps_rxindicate);
	rt_device_open(dev,RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_RX);
	
	rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;
	while(1)
	{
        
		err = rt_sem_take(&gps_sem,RT_WAITING_FOREVER);    //获取信号量
		rt_strncpy((char*)gpsbuf,(char*)rx_fifo->buffer,1024);
		
		if(err == RT_EOK)
		{
//			rt_kprintf("***********退出GPS解析*********\r\n");
			info = Nmea_Decode_para((char*)gpsbuf,1024);
			lat = nmea_ndeg2degree(info.lat);
	        lon = nmea_ndeg2degree(info.lon);
            if(lat!=0 && lon!=0)
			{
				deg_lat = (float)lat;
				deg_lon = (float)lon;
				rt_kprintf("\r\n纬度：%d,经度%d\r\n",deg_lat,deg_lat);
				rt_event_send(&nb_event,GPS);    
				rt_thread_delay(1000 * 60 * 4); //1h
				rt_kprintf("***********退出GPS线程*********\r\n");
			}
			rt_memset(gpsbuf,0,1024);

		}
	}
	
}






















