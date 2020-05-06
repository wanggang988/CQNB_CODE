#include <rtthread.h>

#include "cm_datacom.h"
#include "at.h"
#include <string.h>
#include <stdio.h>
#include "drv_gpio.h"
#include <drv_lptim.h>
#include "devtype.h"

#define LOG_TAG "nbiot_app"
#define LOG_LVL DBG_LOG
#include "ulog.h"			//	必须在LOG_TAG和LOG_LVL下面

#define mipex_buflen    1024
#define  QLWEVTIND  1<<15
#define  NMSTATUS   1<<16
#define AT_CLIENT_RECV_BUFF_LEN   512
#define AT_DEFAULT_TIMEOUT        5000
#define AT_OK                     "OK"
#define AT_ERROR                  "ERROR"
struct rt_event nb_event;   //事件控制块
uint8_t cm_senddata[512];

extern float monitor_value[2];
extern float deg_lat,deg_lon;
extern float deg_x,deg_y;
//extern struct rt_timer otatimer;
extern uint8_t bat_q;
extern float mudhigh;
extern struct cm_mainmsg config_data;
extern struct cm_header cm_default_config;
extern uint32_t rainvalue;
//int module_error = 0;

//void otatimer_callback(void* parameter);
//extern void adc_vol_sample(void);
void refreshkvalue();
void connect_nb_net(void);
void disconnect_nb_net(void);
static void nb_hardware_reset(void);

/*平台的设定值*/
extern float cmangle;              //角度
extern float cmdist;              //拉绳
extern uint32_t cmcycle;         //上传间隔
extern uint16_t cm_soft_ver;    //软件版本
extern uint8_t cm_hard_ver;     //硬件版本
extern uint16_t cmsample;       //采集周期
extern uint16_t cmrain;         //雨量
extern float mudthreshold;      //泥位
extern uint8_t cmautos,cmautolang; //智能报警
extern float dmqx_angle;          //地面倾斜阈值
extern float surface_angle;     //地表角度阈值 float 4byte
extern float surface_dist;      //地表拉线阈值 float 4byte
extern float wall_angle;        //墙裂缝角度阈值 float 4byte
extern float wall_dist;         //墙裂缝拉线阈值修改 float 4byte
void refreshdata(void);

uint32_t observe_msgid;
uint32_t obs_msgid0;
uint32_t obs_msgid1;
uint32_t discover_msgid;

void fal_read(uint32_t Address,uint32_t *buf,uint32_t length);	
int fal_write(uint32_t Address,uint32_t buf_addr,uint32_t length);
static void kvinit(void);
extern void failreg_thread_entry(void *parameter);
extern void cme50_thread_entry(void *parameter);
struct cm_configkv cm_config_kvalue=
{
//	0,
//	0,
	0,          //cycle
	0,          //softversion
	0,          //sample
	0,          //rain
	0,          //autos
	0,          //autolang
    0,          //mud
    0,          //地面倾斜
    0,          //地表角度阈值
    0,          //地表拉线距离
    0,          //墙角度阈值
    0,          //墙角度阈值
};


int csq;    //信号质量
 
struct rt_event nb_event;   //事件控制块
at_response_t resp;     //resp:响应结构体
static int nb_tel_autoconnect();   //NBIOT模组电信自动入网流程函数
static int nb_tel_manconnect();   //NBIOT模组电信手动入网流程函数
static void nb_opennetwork(void);
void nb_send(char* msg,uint16_t len);
static void Hex2Str(char* pSrc, char* pDst, unsigned int nSrcLength);
extern uint8_t crc8_table(uint8_t *ptr, uint8_t len);
static int check_send_cmd(const char* cmd, const char* resp_expr, const rt_size_t lines, const rt_int32_t timeout);
/**
 * This function will send command and check the result.
 *
 * @param cmd       command to at client
 * @param resp_expr expected response expression
 * @param lines     response lines
 * @param timeout   waiting time
 *
 * @return match successful return RT_EOK, otherwise return error code
 */
static int check_send_cmd(const char* cmd, const char* resp_expr, 
                          const rt_size_t lines, const rt_int32_t timeout)
{
    at_response_t resp = RT_NULL;
    int result = 0;

    resp = at_create_resp(AT_CLIENT_RECV_BUFF_LEN, lines, rt_tick_from_millisecond(timeout));
    if (resp == RT_NULL)
    {
        LOG_E("No memory for response structure!");
        return -RT_ENOMEM;
    }

    result = at_exec_cmd(resp, cmd);
    if (result != RT_EOK)
    {
        LOG_E("AT client send commands failed or return response error!");
        at_delete_resp(resp);
        return result;
    }

#if 1
    /* Print response line buffer */
//    char *line_buffer = RT_NULL;
    const char * line_buffer = RT_NULL;

    for(rt_size_t line_num = 1; line_num <= resp->line_counts; line_num++)
    {
        if((line_buffer = at_resp_get_line(resp, line_num)) != RT_NULL)
            LOG_D("line %d buffer : %s", line_num, line_buffer);
        else
            LOG_D("Parse line buffer error!");
    }
#endif

    char resp_arg[AT_CMD_MAX_LEN] = { 0 };
    if (at_resp_parse_line_args_by_kw(resp, resp_expr, "%s", resp_arg) <= 0)
    {
        at_delete_resp(resp);
        LOG_E("# >_< Failed");
        return -RT_ERROR;
    }

    LOG_D("# ^_^ successed");
    at_delete_resp(resp);
    return RT_EOK;
}
/* nb haraware reset 模块上电后，系统自动复位 */
static void nb_hardware_reset(void)
{
    rt_pin_write(NB_EN_PIN,PIN_LOW); 
    rt_thread_delay(1000 * 3);   //delay 3s
    rt_pin_write(NB_EN_PIN,PIN_HIGH); 
}

static void  urc_cfun_func(const char *data, rt_size_t size)
{
	rt_kprintf("打开射频\n");
	rt_event_send(&nb_event,cfun);
}

static void  urc_cgatt_func(const char *data, rt_size_t size)
{
	rt_kprintf("附着上网络\n");
	rt_event_send(&nb_event,CGATT);
}

static void  urc_cereg_func(const char *data, rt_size_t size)
{
	rt_kprintf("注册上网络\n");
	rt_event_send(&nb_event,CEREG);
}


static void urc_nconfig_func(const char *data, rt_size_t size)
{
	 rt_kprintf("配置为默认自动\n");
}

static void urc_error50_func(const char *data, rt_size_t size)
{
    rt_kprintf("Incorrect parameters\r\n");
    rt_event_send(&nb_event,CME50);
//    module_error = 1;
}

static void urc_event_func(const char *data, rt_size_t size)
{
    int i;
    sscanf(data,"+MIPLEVENT: 0,%d",&i);
    rt_kprintf("i = %d\n",i);
    if(i == 6)              //event_reg_success
    { 
        rt_event_send(&nb_event,EVENT); 
    }
    if(i == 7)              //event_reg_failed
    { 
        rt_event_send(&nb_event,FAIL_REG_NET); 
    }
//    if(i == 15)            //event_dereg_done
//    { 
//        rt_event_send(&nb_event,FAIL_REG_NET); 
//    }
    if(i == 20)           //event_response_failed
    { 
        rt_event_send(&nb_event,FAIL_REG_NET); 
    }
    if(i == 25)          //event_notify_failed
    { 
        rt_event_send(&nb_event,FAIL_REG_NET);
//        module_error = 1;
    }
}

static void urc_miplobserve_func(const char *data, rt_size_t size)
{
    int i;
    sscanf(data,"+MIPLOBSERVE: 0,%d,1,3200,%d,",&observe_msgid,&i);
    if(i == 0)    //实例0，上报数据
    {
        obs_msgid0 = observe_msgid; 
        rt_kprintf("obs_msgid0 = %d\n",obs_msgid0);        
    }
   if(i == 1)    //实例1，对平台下发命令的反馈，升级使用
    {
        obs_msgid1 = observe_msgid; 
        rt_kprintf("obs_msgid1 = %d\n",obs_msgid1);         
    }
    rt_event_send(&nb_event,Observe_msgid); 
}
static void urc_mipldiscover_func(const char *data, rt_size_t size)
{
    sscanf(data,"+MIPLDISCOVER: 0,%d,",&discover_msgid);
    rt_kprintf("discover_msgid = %d\n",discover_msgid);
    rt_event_send(&nb_event,Discover_msgid); 
}

static void urc_miplexecute_func(const char *data, rt_size_t size)
{
    int i = 0;
    int execute_msgid = 0;
    int execute_msg_length = 0;
    uint8_t execute_message[1024] = {0};
    uint8_t em_buf[1024] = {0};
    uint8_t mipex_buf[mipex_buflen] = {0};   //每次赋初值0
    uint16_t buff_len = 0;
    sscanf(data,"+MIPLEXECUTE: 0,%d,3200,0,5505,%d,%s\"",&execute_msgid,&execute_msg_length,execute_message);
    rt_kprintf("execute_msgid = %d\n",execute_msgid);
    rt_kprintf("execute_msgidlength = %d\n",execute_msg_length);
    at_exec_cmd(RT_NULL,"AT+MIPLEXECUTERSP=0,%d,2",execute_msgid);      //命令回执
    rt_kprintf("execute_message = %s\n",execute_message);
    buff_len = (uint16_t)(execute_msg_length / 2);
    for(i = 0 ;i<execute_msg_length;i++)
    {
        em_buf[i]  = execute_message[i + 1];             //去除两个""
    }
    str2hex((char*)em_buf,mipex_buf,buff_len);
    if(buff_len > mipex_buflen )
    {
        rt_kprintf("MIPLEXECUTERSP is overflow\r\n");
        memset(mipex_buf,0,mipex_buflen);
    }
    cm_decode(buff_len,mipex_buf);
}
static void urc_qlwevtind_func(const char *data, rt_size_t size)
{
	rt_kprintf("订阅对象19/0/0完成\r\n");
	rt_event_send(&nb_event,QLWEVTIND);         
}
static void urc_nmstatus_func(const char *data, rt_size_t size)
{
	rt_kprintf("UE可以发送数据!\r\n");
	rt_event_send(&nb_event,NMSTATUS);         
}
#define urc_table_size 11
static struct at_urc urc_table[urc_table_size] = {
    {"+CFUN:1","\r\n", urc_cfun_func},
	{"+CGATT:1", "\r\n", urc_cgatt_func},
	{"+CEREG:0,1", "\r\n", urc_cereg_func},
	{"+NCONFIG:AUTOCONNECT,TRUE", "\r\n", urc_nconfig_func},
//	{"+", "NNMI", urc_nnmi_func},

    {"+MIPLEVENT: 0,","\r\n",urc_event_func},
    {"+MIPLOBSERVE: 0,","\r\n",urc_miplobserve_func},
    {"+MIPLDISCOVER: 0,","\r\n",urc_mipldiscover_func},
    {"+MIPLEXECUTE: 0,","\r\n",urc_miplexecute_func},
    {"+CME ERROR: 50", "\r\n", urc_error50_func},
    {"+QLWEVTIND:3","\r\n",urc_qlwevtind_func},
    {"+NMSTATUS:MO_DATA_ENABLED","\r\n",urc_nmstatus_func},
//    {"+MIPLEVENT: 0,20","\r\n",urc_response_event_func},
//    {"+MIPLEVENT: 0,25","\r\n",urc_notify_event_func},
//    {"+MIPLEVENT: 0,7","\r\n",urc_reg_event_func},
};


extern float deg_lat,deg_lon;
extern struct cm_header cm_default_config;          //header
extern struct cm_mainmsg config_data;               //第二层数据
extern uint16_t liqhigh;
void nb_up_heartbeat(cm_header_t ptr,uint16_t length)
{
    config_data.cmd = cm_heartbeat_up;    //上传心跳包
    #if USING_DBLH
    cm_default_config.dev_type = CM_EARTH_DEV;     //设备类型改为地表裂缝监测仪
    cm_default_config.dev_id = CM_DBLF_ID;
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报心跳包
    #endif

    #if USING_QLH
    cm_default_config.dev_type = CM_WALL_DEV;     //设备类型改为墙裂缝监测仪
    cm_default_config.dev_id = CM_QLF_ID;
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报心跳包
    #endif

    #if USING_DMQX
    cm_default_config.dev_type = CM_ANGLE_DEV;     //设备类型改为地面倾斜监测仪
    cm_default_config.dev_id = CM_DMQX_ID;         //设备ID改为地面倾斜监测仪
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报心跳包
    #endif

    #if USING_MUD
    cm_default_config.dev_type = CM_MUD_DEV;     //设备类型改为泥位计
    cm_default_config.dev_id = CM_MUD_ID;         //设备ID改为泥位计
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报心跳包
    #endif

    #if USING_RAIN
    cm_default_config.dev_type = CM_RAIN_DEV;     //设备类型雨量计
    cm_default_config.dev_id = CM_RAIN_ID;     
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报心跳包
    #endif

    #if USING_ALARMER
    cm_default_config.dev_type = CM_AUTO_DEV;     //设备类型改为智能报警器
    cm_default_config.dev_id = CM_ANNU_ID;     
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报心跳包
    #endif    
}    

void failreg_thread_entry(void *parameter)
{
	while(1)
	{
		rt_event_recv(&nb_event,FAIL_REG_NET,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR ,RT_WAITING_FOREVER, RT_NULL);
		rt_kprintf("进入NB网络异常处理线程\n");
//        rt_enter_critical();
		disconnect_nb_net();
		connect_nb_net();
		rt_kprintf("退出NB网络异常处理线程\n");
//        module_error = 0;
//        rt_exit_critical();
		rt_thread_delay(1000 * 5);  // delay 5s
		
	}
	
}
void cme50_thread_entry(void *parameter)
{
	while(1)
	{
		rt_event_recv(&nb_event,CME50,RT_EVENT_FLAG_OR |RT_EVENT_FLAG_CLEAR ,RT_WAITING_FOREVER, RT_NULL);
//        rt_event_recv(&nb_event,NBNET,RT_EVENT_FLAG_OR,RT_WAITING_FOREVER, RT_NULL);
		rt_kprintf("进入模块异常处理线程,系统复位\r\n");
        HAL_NVIC_SystemReset();//system reset

		rt_thread_delay(1000 * 5);  //delay 5s
		
	}
	
}
void nb_thread_entry(void *parameter)
{
    cm_header_t ptr = &cm_default_config;	 //header
    uint16_t length;
//    rt_timer_init(&otatimer,"otatim",otatimer_callback,RT_NULL,1000 * 20,RT_TIMER_FLAG_ONE_SHOT);//静态定时器进行初始化
	rt_event_init(&nb_event,"nb",RT_IPC_FLAG_FIFO); //初始化事件集
	at_set_urc_table(urc_table,urc_table_size);
    rt_thread_delay(1000*5);
	kvinit();   //初始值flash
//    nb_tel_autoconnect();
    nb_tel_manconnect();
//	nb_opennetwork();
//    adc_vol_sample();          //adc sample
    refreshdata();  
    rt_event_recv(&nb_event,GPS,RT_EVENT_FLAG_OR,1000 * 60 * 3, RT_NULL); //3min for gps
//    while(module_error == 1);
    cm_device_init(ptr,length);
	while(1)
	{
        rainvalue = 0;
        refreshdata();
        rt_kprintf("进入上传模块信息线程\n");
        cm_default_config.dev_type = CM_EARTH_DEV;     //地表
        cm_default_config.dev_id = CM_DBLF_ID;  
        config_data.cmd = cm_modulestatus_up;  
        length = cm_encode(ptr,&config_data);
        nb_send((char*)cm_senddata,length); 			
		rt_kprintf("退出上传模块信息线程\n");
//        rt_pm_request(PM_SLEEP_MODE_DEEP);    //请求进入深度睡眠模式
        rt_thread_delay(cmcycle); 
        
	}
}

void heart_thread_entry(void *parameter)
{
	cm_header_t ptr = &cm_default_config;
	uint16_t l;
	rt_event_recv(&nb_event,NBNET,RT_EVENT_FLAG_OR,RT_WAITING_FOREVER, RT_NULL);
    rt_thread_delay(1000 * 60 *2); //5分钟上传一次心跳包 cmcycle为设定的上报周期
	while(1)
	{
		rt_kprintf("进入heartbeat线程\n");
        refreshdata();
        config_data.cmd = cm_heartbeat_up;    //上传心跳包
        cm_default_config.dev_type = CM_EARTH_DEV;     //设备类型改为墙裂缝监测仪
        cm_default_config.dev_id = CM_DBLF_ID;
        l = cm_encode(ptr,&config_data);
//        while(module_error == 1);
        nb_send((char*)cm_senddata,l);  //上报心跳包
		rt_kprintf("退出heartbeat线程\n");
//        rt_pm_request(PM_SLEEP_MODE_DEEP);    //请求进入深度睡眠模式
		rt_thread_delay(1000 * 60* 60 * 1); //1h
	}
}

char *life="AA11BB";
void life_thread_entry(void *parameter)
{
	rt_event_recv(&nb_event,NBNET,RT_EVENT_FLAG_OR,RT_WAITING_FOREVER, RT_NULL);
	while(1)
	{
		
		rt_thread_delay(1000 * 60 * 7); //8min
		rt_kprintf("进入life线程\n");
//        while(module_error == 1);
		nb_send(life,3);          //唤醒设备
		rt_kprintf("退出life线程\n");
//        rt_pm_request(PM_SLEEP_MODE_DEEP);    //请求进入深度睡眠模式
	}
}
/* func: connect and open network */
int status;
//static void nb_opennetwork()
//{
//    at_response_t resp;     //resp:响应结构体
//    resp = at_create_resp(512,3,500);
//	int status;
//    uint16_t n;
//    nb_hardware_reset();

//    rt_thread_delay(1000 * 7);
//	at_exec_cmd(resp,"AT+NATSPEED=9600,3,1,0,1,0,0");   // 配置波特率
////	at_exec_cmd(resp,"AT+NBAND=3");   //联通
//	at_exec_cmd(resp,"AT+NBAND=8");   // 移动 可以不配置
////	at_exec_cmd(resp,"AT+NBAND=5");   // 电信

//	if(at_client_wait_connect(1000*5) < 0)   // at_client_wait_connect():等待模块初始化完成
//	{
//		rt_kprintf("连接BC35-G模块失败\n");
//	}
//    rt_thread_delay(1000 * 2);
//    do{
//		at_exec_cmd(resp,"AT+CFUN?");
//        rt_thread_delay(1000);	
//	}while(rt_event_recv(&nb_event,cfun,RT_EVENT_FLAG_OR,1000, RT_NULL) != RT_EOK);
//    do{
//		at_exec_cmd(resp,"AT+NBAND?");
//		at_resp_parse_line_args_by_kw(resp,"+NBAND:","+NBAND:%d",&status); //解析指定关键字行的响应数据
//		rt_thread_delay(1000);
//	}while(status != 8);
//    at_exec_cmd(resp,"AT+CMEE=1");   //报告错误代码
//    at_exec_cmd(resp,"AT+CIMI");     //international mobile subscriber id
//    at_exec_cmd(resp,"AT+CGSN=1");     //获取NBIOT IMEI信息
//    at_exec_cmd(resp,"AT+CPSMS=0");  //disable the use of PSM
//    at_exec_cmd(resp,"AT+NCONFIG?");   //configure UE Behaviour
//    rt_thread_delay(1000 * 3);
//    at_exec_cmd(resp,"AT+COPS=1,2,\"46000\"");         //手动附网
//	do{
//		at_exec_cmd(resp,"AT+CSQ");
//		at_resp_parse_line_args_by_kw(resp,"+CSQ:","+CSQ:%d",&status); //解析指定关键字行的响应数据
//		rt_thread_delay(1000);
//	}while(status > 31);
//	
//	do{
//		at_exec_cmd(resp,"AT+CGATT?");
//		rt_thread_delay(1000);
//        }while(rt_event_recv(&nb_event,CGATT,RT_EVENT_FLAG_OR,1000, RT_NULL) != RT_EOK ); //RT_EVENT_FLAG_OR:逻辑或的方式接收事件
//	
//	do{
//		at_exec_cmd(resp,"AT+CEREG?");
//		rt_thread_delay(1000);
//	}while(rt_event_recv(&nb_event,CEREG,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,1000, RT_NULL) != RT_EOK);

//    at_exec_cmd(resp,"AT+MIPLCONFIG=0,183.230.40.40,5683"); //configure LwM2M server IP and port
//    rt_thread_delay(1000);
//    at_exec_cmd(resp,"AT+MIPLCONFIG?");  //查询  LwM2M server ip and port
//    rt_thread_delay(1000);
//    connect_nb_net();
//    at_delete_resp(resp);    
//}

/* query CSQ */
void bc35_query_csq(void)
{
    int status = 0;
    at_response_t resp;     //resp:响应结构体
    resp = at_create_resp(512,3,500);
    
    do{
		at_exec_cmd(resp,"AT+CSQ");   //查询信号强度
        rt_thread_delay(300);	          //最大相应时间300ms
		at_resp_parse_line_args_by_kw(resp,"+CSQ:","+CSQ:%d",&status); //解析指定关键字行的响应数据
		rt_thread_delay(1000);
	}while(status > 31);    //信号强度大于31,再次查询。
    at_delete_resp(resp);
}
/* query UE'S IP */
void bc35_query_ip(void)
{
    uint8_t count = 0;
    at_response_t resp;     //resp:响应结构体
    resp = at_create_resp(512,3,500);
    do{
        at_exec_cmd(resp,"AT+CGPADDR"); //查询模块是否获取到IP
        rt_thread_delay(300);  //最大相应时间300ms
        if(at_resp_get_line_by_kw(resp,"OK") != NULL)
        {
            LOG_D("模块已分配到IP地址\r\n");
            break;
        }
        rt_thread_delay(1000);  //延时1S再次查询，循环3次结束
        count++;
    }while(count < 3);
    count = 0;  //次数清零
    at_delete_resp(resp);
}
/*以下为电信的自动入网步骤*/
static int nb_tel_autoconnect()
{
    int result = 0;
    uint8_t count = 0;     //resp:命令循环次数
	int status;
    rt_pin_write(NB_EN_PIN,PIN_HIGH);//打开NBIOT模组电源，模块上电
    rt_thread_delay(1000 * 5); //约5S后会输出Neul OK字样，表示模块可以执行AT指令
	if(at_client_wait_connect(1000*5) < 0)   // at_client_wait_connect():等待模块初始化完成
	{
		LOG_D("连接BC35-G模块失败\n");
	}
    /* 报告错误代码 */
    check_send_cmd("AT+CMEE=1", AT_OK, 0, 300);
    if (result != RT_EOK) return result;
    /* 设置UE为自动入网模式 */
    result = check_send_cmd("AT+NCONFIG=AUTOCONNECT,TRUE", AT_OK, 0, 300);
    if (result != RT_EOK) return result;
    /* 复位UE */
    check_send_cmd("AT+NRB", AT_OK, 0, 6000);
    while(RT_EOK != check_send_cmd("AT", AT_OK, 0, AT_DEFAULT_TIMEOUT))
    {
        rt_thread_mdelay(1000);
    }
    /* 检查USIM卡是否初始化成功 */
    do{
        result = check_send_cmd("AT+CIMI", "460", 0, 300);
        if (result == RT_EOK) 
        {
            LOG_D("获取到USIM卡的IMSI号\r\n");
            break;
        }
        rt_thread_delay(1000);  //延时1S再次查询，循环10次结束
        count++;
       
    }while(count < 10);
    count = 0;  //次数清零
    
    /* 获取NBIOT IMEI信息 */
    do{
        result = check_send_cmd("AT+CGSN=1", "+CGSN", 0, 300);
        if (result == RT_EOK) 
        {
            LOG_D("获取到NBIOT模块IMEI信息\r\n");
            break;
        }
        if(result != RT_EOK)
        {
            return result; 
        }
        rt_thread_delay(1000);  //延时1S再次查询，循环10次结束
        count++;
    }while(count < 3);
    count = 0;  //次数清零
    bc35_query_csq();
    /* 查询模块的网络附着状态 */
    do{
		at_exec_cmd(resp,"AT+CEREG?"); //查询模块的网络附着状态
		rt_thread_delay(1000);
	}while(rt_event_recv(&nb_event,CEREG,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,1000, RT_NULL) != RT_EOK);
    /* 查询模块的PDP上下文激活状态 */
    do{
		at_exec_cmd(resp,"AT+CGATT?");//查询模块的PDP上下文激活状态
		rt_thread_delay(1000);
    }while(rt_event_recv(&nb_event,CGATT,RT_EVENT_FLAG_OR,1000, RT_NULL) != RT_EOK ); //RT_EVENT_FLAG_OR:逻辑或的方式接收事件
    /* query UE'S IP Address */
    bc35_query_ip();
    /* 设置指示和消息 */
    result = check_send_cmd("AT+NNMI=1", AT_OK, 0, 300);
    if (result != RT_EOK) return result;
    /* 当模块连接到CDP服务器时，该命令用来上报当前注册状态 */
    do{
		result = check_send_cmd("AT+NMSTATUS?", "+NMSTATUS", 0, 300); 
		rt_thread_delay(1000);
	}while(rt_event_recv(&nb_event,NMSTATUS,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,1000, RT_NULL) != RT_EOK);  
}
/*以下为电信的手动入网步骤*/
static int nb_tel_manconnect()
{
    int result = 0;
    uint8_t count = 0;     //命令循环次数
	int status;
    rt_pin_write(NB_EN_PIN,PIN_HIGH);//打开NBIOT模组电源，模块上电
    rt_thread_delay(1000 * 5); //约5S后会输出Neul OK字样，表示模块可以执行AT指令
    if(at_client_wait_connect(1000*5) < 0)   // at_client_wait_connect():等待模块初始化完成
	{
		LOG_D("连接BC35-G模块失败\n");
	}
    /* 报告错误代码 */
    check_send_cmd("AT+CMEE=1", AT_OK, 0, 300);
    if (result != RT_EOK) return result;
    /* 设置UE为手动入网模式 */
    result = check_send_cmd("AT+NCONFIG=AUTOCONNECT,FALSE", AT_OK, 0, 300);
    if (result != RT_EOK) return result;
    /* 设置CDP服务器*/
    do{
        result = check_send_cmd("AT+NCDP=119.3.250.80,5683",AT_OK,0,300); //华为云调试接口iot-coaps.cn-north-4.myhuaweicloud.com
        if (result == RT_EOK) 
        {
            LOG_D("CDP服务器设置成功\r\n");
            break;
        }
        rt_thread_delay(1000);  //延时1S再次查询，循环3次结束
        count++;
    }while(count < 3);
    count = 0;  //次数清零
    /* 复位UE */
    check_send_cmd("AT+NRB", AT_OK, 0, 6000);
    while(RT_EOK != check_send_cmd("AT", AT_OK, 0, AT_DEFAULT_TIMEOUT))
    {
        rt_thread_mdelay(1000);
    }
    /* 打开射频开关 */
    do{
        result = check_send_cmd("AT+CFUN=1", AT_OK, 0, 300);
        if (result == RT_EOK) 
        {
            LOG_D("射频开关已打开\r\n");
            break;
        }
        rt_thread_delay(1000);  //延时1S再次查询，循环5次结束
        count++;
    }while(count < 5);
    count = 0;  //次数清零
    rt_thread_delay(5000);  //延时5S再次查询
    /* 检查USIM卡是否初始化成功 */
    do{
        result = check_send_cmd("AT+CIMI", "460", 0, 300);
        if (result == RT_EOK) 
        {
            LOG_D("获取到USIM卡的IMSI号\r\n");
            break;
        }
        rt_thread_delay(1000);  //延时1S再次查询，循环10次结束
        count++;
    }while(count < 10);
    count = 0;  //次数清零
    /* 获取NBIOT IMEI信息 */
    do{
        result = check_send_cmd("AT+CGSN=1", "+CGSN", 0, 300);
        if (result == RT_EOK) 
        {
            LOG_D("获取到NBIOT模块IMEI信息\r\n");
            break;
        }
        rt_thread_delay(1000);  //延时1S再次查询，循环10次结束
        count++;
    }while(count < 3);
    count = 0;  //次数清零
    /* PS附着 */
    do{
        result = check_send_cmd("AT+CGATT=1", AT_OK, 0, 1000);
        if (result == RT_EOK) 
        {
            LOG_D("PS附着设置成功\r\n");
            break;
        }
        rt_thread_delay(1000);  //延时1S再次查询，循环3次结束
        count++;
    }while(count < 3);
    count = 0;  //次数清零
    bc35_query_csq();
    do{
		at_exec_cmd(resp,"AT+CEREG?"); //查询模块的网络附着状态
		rt_thread_delay(1000);
	}while(rt_event_recv(&nb_event,CEREG,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,1000, RT_NULL) != RT_EOK);
    do{
		at_exec_cmd(resp,"AT+CGATT?");//查询模块的PDP上下文激活状态
		rt_thread_delay(1000);
    }while(rt_event_recv(&nb_event,CGATT,RT_EVENT_FLAG_OR,1000, RT_NULL) != RT_EOK ); //RT_EVENT_FLAG_OR:逻辑或的方式接收事件
    /* query UE'S IP Address */
    bc35_query_ip();
    /* 设置指示和消息 */
    result = check_send_cmd("AT+NNMI=1", AT_OK, 0, 300);
    if (result != RT_EOK) return result;
    /* 当模块连接到CDP服务器时，该命令用来上报当前注册状态 */
    do{
		result = check_send_cmd("AT+NMSTATUS?", "+NMSTATUS", 0, 300); 
		rt_thread_delay(1000);
	}while(rt_event_recv(&nb_event,NMSTATUS,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,1000, RT_NULL) != RT_EOK);  
}
uint32_t ackid = 1;
uint32_t ackid1 = 1;
/* 移动 */
//void nb_send(char* msg, uint16_t len)
//{
//    char send[512];
////    char cmdresult[5] = {0};
//    at_response_t resp;     //resp:响应结构体
//    resp = at_create_resp(512,3,500);
//    Hex2Str(msg,send,len);
//    at_exec_cmd(resp,"AT+MIPLNOTIFY=0,%d,3200,0,5505,2,%d,%s,0,0,%d",obs_msgid0,len,send,ackid); //对象实例0 移动
//   
//    while(at_resp_parse_line_args(resp,2,"%s","OK") > 0)//解析成功
//    {
//        rt_kprintf("上报数据成功,完成本次任务\n");
//        break;        
//    }
//    while(at_resp_parse_line_args(resp,1,"%s","ERROR") > 0)//解析成功
//    {
//        rt_kprintf("上报数据失败,结束本次任务\n");
//        rt_event_recv(&nb_event,NBNET,RT_EVENT_FLAG_OR,RT_WAITING_FOREVER, RT_NULL); //在联网的前提下
//        at_exec_cmd(RT_NULL,"AT+MIPLUPDATE=0,86400,0");  //update lifetime
//        rt_kprintf("更新生命周期成功\n"); 
//        break;         
//    }
//    ackid++;
//    if(ackid > 65535)
//    {
//        ackid = 1;
//    }
//    at_delete_resp(resp);	
//}
/*华为云平台*/
void nb_send(char* msg, uint16_t len)
{
    char send[512];
    at_response_t resp;     //resp:响应结构体
    resp = at_create_resp(512,3,500);
    Hex2Str(msg,send,len);
    at_exec_cmd(resp,"AT+NMGS=%d,%s",len,send); //上报数据
   
    while(at_resp_parse_line_args(resp,2,"%s","OK") > 0)//解析成功
    {
        rt_kprintf("上报数据成功,完成本次任务\n");
        break;        
    }
    at_delete_resp(resp);	
}
//void nb_send_ota(char* msg, uint16_t len)
//{
//    char send[512];
//    at_response_t resp;     //resp:响应结构体
//    resp = at_create_resp(512,3,500);
//    Hex2Str(msg,send,len);
//    at_exec_cmd(RT_NULL,"AT+MIPLNOTIFY=0,%d,3200,1,5505,2,%d,%s,0,0,%d",obs_msgid1,len,send,ackid1); //对象实例1
//    ackid1++;
//    if(ackid1 > 65535)
//    {
//        ackid1 = 1;
//    }
//    while(at_resp_parse_line_args(resp,2,"%s","OK") > 0)//解析成功
//    {
//        rt_kprintf("上报数据成功,完成本次任务\n");
//        break;
//    }
//    while(at_resp_parse_line_args(resp,2,"%s","ERROR") > 0)//解析成功
//    {
//        rt_kprintf("上报数据失败,结束本次任务\n");
//        rt_event_recv(&nb_event,NBNET,RT_EVENT_FLAG_OR,RT_WAITING_FOREVER, RT_NULL); //在联网的前提下
//        at_exec_cmd(RT_NULL,"AT+MIPLUPDATE=0,86400,0");  //update lifetime
//        rt_kprintf("更新生命周期成功\n");
//        break;
//    }
//    at_delete_resp(resp);
//}

static void Hex2Str(char* pSrc, char* pDst, unsigned int nSrcLength)
{
    int i = 0;
    const char tab[]="0123456789ABCDEF";    // 0x0-0xf的字符查找表

    for (i = 0; i < nSrcLength; i++)
    {
        *pDst++ = tab[*pSrc >> 4];      // 输出高4位
        *pDst++ = tab[*pSrc & 0x0f];    // 输出低4位
        pSrc++;
    }

    // 输出字符串加个结束符
    *pDst = '\0';
}


void refreshdata(void)
{
	uint32_t rand;
	float randf;
	at_response_t resp;
	resp = at_create_resp(512,3,500);
	
	config_data.angle_x = deg_x;      //传感器读取的X轴值
	config_data.angle_y = deg_y;      //传感器读取的Y轴值
	
	config_data.dist = monitor_value[1];     //拉绳
	config_data.bat_per = bat_q;           //电量
	config_data.bat_vol = monitor_value[0];    //电压
	
	config_data.gps_lat = deg_lat;      
	config_data.gps_long = deg_lon; 
	rand = rt_tick_get();
	randf =  (float)(rand%100)/1000000;
	config_data.bd_lat = deg_lat + randf; 
	rand = rt_tick_get();
	randf =  (float)((rand * rand)%100)/1000000;
	config_data.bd_long = deg_lon + randf;
    
    config_data.mud_n = (float)liqhigh;   //位高值单位mm
	config_data.rain_n = (float)(rainvalue * 0.1);  //单位mm
    config_data.auto_s = cmautos;   
	config_data.auto_lang  = cmautolang;  
                                       
	at_exec_cmd(resp,"AT+CSQ");
	at_resp_parse_line_args_by_kw(resp,"+CSQ:","+CSQ:%d",&csq);
	config_data.signal = csq;
	at_delete_resp(resp);
}

void connect_nb_net(void)
{
	at_response_t resp;     //resp:响应结构体
	resp = at_create_resp(512,3,500);
	at_exec_cmd(resp,"AT+MIPLCREATE"); //对接移动ONEnet平台，创建基础通讯套件实例
	rt_thread_delay(1000 * 1);
	at_exec_cmd(resp,"AT+MIPLADDOBJ=0,3200,2,\"11\",1,1"); //添加对象和实例,对接移动ONEnet平台，ONENET设备实例ID:0,添加3200(ISPO数字量输入) 类型对象，2个实例
	rt_thread_delay(1000 * 1);
	at_exec_cmd(resp,"AT+MIPLOPEN=0,86400,60");// 登录ONENET平台   lifetime = 86400s = 1day
	rt_thread_delay(1000 * 1);
//	if(rt_event_recv(&nb_event,EVENT,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, RT_NULL) == RT_EOK)
	if(rt_event_recv(&nb_event,EVENT,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,1000 * 60 * 15, RT_NULL) == RT_EOK) //15min
	{
			rt_kprintf("联网成功\n");    
	}
//	if(rt_event_recv(&nb_event,Observe_msgid,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, RT_NULL) == RT_EOK)
	if(rt_event_recv(&nb_event,Observe_msgid,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,1000 * 60 * 15, RT_NULL) == RT_EOK)	//15min
	{
//			if(rt_event_recv(&nb_event,Discover_msgid,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, RT_NULL) == RT_EOK)
			if(rt_event_recv(&nb_event,Discover_msgid,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,1000 * 60 * 15, RT_NULL) == RT_EOK)		//15min
			{

					at_exec_cmd(resp,"AT+MIPLDISCOVERRSP=0,%d,1,4,\"5505\"",discover_msgid);
//					rt_thread_delay(300);
			}
	}
    rt_event_send(&nb_event,NBNET); //发送网络已连接事件
	at_delete_resp(resp);

	
}
void disconnect_nb_net(void)
{
	at_response_t resp;     //resp:响应结构体
	resp = at_create_resp(512,3,500);
	at_exec_cmd(resp,"AT+MIPLDELOBJ=0,3200"); //Delete a LWM2M object
	at_exec_cmd(resp,"AT+MIPLCLOSE=0"); 
	at_exec_cmd(resp,"AT+MIPLDELETE=0");
	at_delete_resp(resp);	
}

/*刷新数据*/
void refreshkvalue()
{
	uint32_t *raddr,waddr;
	raddr = (uint32_t*)&cm_config_kvalue;
	fal_read(0,raddr,sizeof(struct cm_configkv)/8);     //读flash KV区
	cm_config_kvalue.surface_ang = surface_angle;
    cm_config_kvalue.surface_dis = surface_dist;
    cm_config_kvalue.wall_ang = wall_angle;
    cm_config_kvalue.wall_dis = wall_dist;
	cm_config_kvalue.autos = (uint32_t)cmautos;
	cm_config_kvalue.autolang = (uint32_t)cmautolang;
	cm_config_kvalue.rain = (uint32_t)cmrain;
    cm_config_kvalue.mud = mudthreshold; 
	cm_config_kvalue.sample = (uint32_t)cmsample;
	cm_config_kvalue.cycle = (uint32_t)cmcycle;
    cm_config_kvalue.dmqx_angle = dmqx_angle;
	
	if(cm_config_kvalue.softversion != 0xFFFFFFFF)
	{
		if(cm_config_kvalue.softversion < (uint32_t)cm_soft_ver)
		{
			cm_config_kvalue.softversion = (uint32_t)cm_soft_ver;
		}
		else                                                 //当前版本大于之前的版本，
		{
			cm_soft_ver =(uint16_t) cm_config_kvalue.softversion;
		}
	}
	else
	{
		cm_config_kvalue.softversion = (uint32_t)cm_soft_ver;
	}
	
	waddr = (uint32_t)&cm_config_kvalue;
	fal_write(0,waddr,sizeof(struct cm_configkv)/8);          //写flahs  KV区
    rt_kprintf("flash ok\n");
}


static void kvinit()
{
	uint32_t *raddr;
	raddr = (uint32_t*)&cm_config_kvalue;
	fal_read(0,raddr,sizeof(struct cm_configkv)/4);     //读flash KV区; 
	surface_angle = cm_config_kvalue.surface_ang;
	surface_dist = cm_config_kvalue.surface_dis;
	wall_angle = cm_config_kvalue.wall_ang;
	wall_dist = cm_config_kvalue.wall_dis;
	cmautos =(uint8_t) cm_config_kvalue.autos;
	cmautolang = (uint8_t)cm_config_kvalue.autolang;
	cmrain =(uint16_t) cm_config_kvalue.rain;
    mudthreshold = cm_config_kvalue.mud;
	cmsample =(uint16_t) cm_config_kvalue.sample;
	cmcycle = cm_config_kvalue.cycle;
    dmqx_angle = cm_config_kvalue.dmqx_angle; 
	if(cm_config_kvalue.softversion != 0xFFFFFFFF)    //不等于0XFFFFFFFF 说明flash中版本已经被写入。
	{
		if(cm_config_kvalue.softversion < (uint32_t)cm_soft_ver)
		{
			cm_config_kvalue.softversion = (uint32_t)cm_soft_ver;
		}
		else                                                           //升级了新版本
		{
            cm_soft_ver = (uint16_t)cm_config_kvalue.softversion;
		}
	}
	else                                                   //等于0XFFFFFFFF 说明flash中版本未被写入。
	{
        cm_config_kvalue.softversion = (uint32_t)cm_soft_ver; //将初始值版本写进去。
        
	}
}

static void firstrefreshkvalue()
{
    rt_kprintf("\r refresh kvalue ok!\r\n");
    uint32_t *raddr,waddr;
    raddr = (uint32_t*)&cm_config_kvalue;
    fal_read(0,raddr,sizeof(struct cm_configkv)/4);     //读flash KV区
  
    cm_config_kvalue.surface_ang = surface_angle;
    cm_config_kvalue.surface_dis = surface_dist;
    cm_config_kvalue.wall_ang = wall_angle;
    cm_config_kvalue.wall_dis = wall_dist;
    cm_config_kvalue.dmqx_angle = dmqx_angle;
    cm_config_kvalue.autos = cmautos;
    cm_config_kvalue.autolang = cmautolang;
    cm_config_kvalue.rain = cmrain;
    cm_config_kvalue.mud = mudthreshold;
    cm_config_kvalue.sample = cmsample ;
    cm_config_kvalue.cycle = cmcycle;
    cm_config_kvalue.softversion = (uint32_t)cm_soft_ver;
    waddr = (uint32_t)&cm_config_kvalue;
    fal_write(0,waddr,sizeof(struct cm_configkv)/8);          //写flahs  KV区
}
MSH_CMD_EXPORT(firstrefreshkvalue, refresh first kvalue);