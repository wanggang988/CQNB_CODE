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
#include "ulog.h"			//	������LOG_TAG��LOG_LVL����

#define mipex_buflen    1024
#define  QLWEVTIND  1<<15
#define  NMSTATUS   1<<16
#define AT_CLIENT_RECV_BUFF_LEN   512
#define AT_DEFAULT_TIMEOUT        5000
#define AT_OK                     "OK"
#define AT_ERROR                  "ERROR"
struct rt_event nb_event;   //�¼����ƿ�
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

/*ƽ̨���趨ֵ*/
extern float cmangle;              //�Ƕ�
extern float cmdist;              //����
extern uint32_t cmcycle;         //�ϴ����
extern uint16_t cm_soft_ver;    //����汾
extern uint8_t cm_hard_ver;     //Ӳ���汾
extern uint16_t cmsample;       //�ɼ�����
extern uint16_t cmrain;         //����
extern float mudthreshold;      //��λ
extern uint8_t cmautos,cmautolang; //���ܱ���
extern float dmqx_angle;          //������б��ֵ
extern float surface_angle;     //�ر�Ƕ���ֵ float 4byte
extern float surface_dist;      //�ر�������ֵ float 4byte
extern float wall_angle;        //ǽ�ѷ�Ƕ���ֵ float 4byte
extern float wall_dist;         //ǽ�ѷ�������ֵ�޸� float 4byte
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
    0,          //������б
    0,          //�ر�Ƕ���ֵ
    0,          //�ر����߾���
    0,          //ǽ�Ƕ���ֵ
    0,          //ǽ�Ƕ���ֵ
};


int csq;    //�ź�����
 
struct rt_event nb_event;   //�¼����ƿ�
at_response_t resp;     //resp:��Ӧ�ṹ��
static int nb_tel_autoconnect();   //NBIOTģ������Զ��������̺���
static int nb_tel_manconnect();   //NBIOTģ������ֶ��������̺���
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
/* nb haraware reset ģ���ϵ��ϵͳ�Զ���λ */
static void nb_hardware_reset(void)
{
    rt_pin_write(NB_EN_PIN,PIN_LOW); 
    rt_thread_delay(1000 * 3);   //delay 3s
    rt_pin_write(NB_EN_PIN,PIN_HIGH); 
}

static void  urc_cfun_func(const char *data, rt_size_t size)
{
	rt_kprintf("����Ƶ\n");
	rt_event_send(&nb_event,cfun);
}

static void  urc_cgatt_func(const char *data, rt_size_t size)
{
	rt_kprintf("����������\n");
	rt_event_send(&nb_event,CGATT);
}

static void  urc_cereg_func(const char *data, rt_size_t size)
{
	rt_kprintf("ע��������\n");
	rt_event_send(&nb_event,CEREG);
}


static void urc_nconfig_func(const char *data, rt_size_t size)
{
	 rt_kprintf("����ΪĬ���Զ�\n");
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
    if(i == 0)    //ʵ��0���ϱ�����
    {
        obs_msgid0 = observe_msgid; 
        rt_kprintf("obs_msgid0 = %d\n",obs_msgid0);        
    }
   if(i == 1)    //ʵ��1����ƽ̨�·�����ķ���������ʹ��
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
    uint8_t mipex_buf[mipex_buflen] = {0};   //ÿ�θ���ֵ0
    uint16_t buff_len = 0;
    sscanf(data,"+MIPLEXECUTE: 0,%d,3200,0,5505,%d,%s\"",&execute_msgid,&execute_msg_length,execute_message);
    rt_kprintf("execute_msgid = %d\n",execute_msgid);
    rt_kprintf("execute_msgidlength = %d\n",execute_msg_length);
    at_exec_cmd(RT_NULL,"AT+MIPLEXECUTERSP=0,%d,2",execute_msgid);      //�����ִ
    rt_kprintf("execute_message = %s\n",execute_message);
    buff_len = (uint16_t)(execute_msg_length / 2);
    for(i = 0 ;i<execute_msg_length;i++)
    {
        em_buf[i]  = execute_message[i + 1];             //ȥ������""
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
	rt_kprintf("���Ķ���19/0/0���\r\n");
	rt_event_send(&nb_event,QLWEVTIND);         
}
static void urc_nmstatus_func(const char *data, rt_size_t size)
{
	rt_kprintf("UE���Է�������!\r\n");
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
extern struct cm_mainmsg config_data;               //�ڶ�������
extern uint16_t liqhigh;
void nb_up_heartbeat(cm_header_t ptr,uint16_t length)
{
    config_data.cmd = cm_heartbeat_up;    //�ϴ�������
    #if USING_DBLH
    cm_default_config.dev_type = CM_EARTH_DEV;     //�豸���͸�Ϊ�ر��ѷ�����
    cm_default_config.dev_id = CM_DBLF_ID;
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //�ϱ�������
    #endif

    #if USING_QLH
    cm_default_config.dev_type = CM_WALL_DEV;     //�豸���͸�Ϊǽ�ѷ�����
    cm_default_config.dev_id = CM_QLF_ID;
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //�ϱ�������
    #endif

    #if USING_DMQX
    cm_default_config.dev_type = CM_ANGLE_DEV;     //�豸���͸�Ϊ������б�����
    cm_default_config.dev_id = CM_DMQX_ID;         //�豸ID��Ϊ������б�����
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //�ϱ�������
    #endif

    #if USING_MUD
    cm_default_config.dev_type = CM_MUD_DEV;     //�豸���͸�Ϊ��λ��
    cm_default_config.dev_id = CM_MUD_ID;         //�豸ID��Ϊ��λ��
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //�ϱ�������
    #endif

    #if USING_RAIN
    cm_default_config.dev_type = CM_RAIN_DEV;     //�豸����������
    cm_default_config.dev_id = CM_RAIN_ID;     
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //�ϱ�������
    #endif

    #if USING_ALARMER
    cm_default_config.dev_type = CM_AUTO_DEV;     //�豸���͸�Ϊ���ܱ�����
    cm_default_config.dev_id = CM_ANNU_ID;     
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //�ϱ�������
    #endif    
}    

void failreg_thread_entry(void *parameter)
{
	while(1)
	{
		rt_event_recv(&nb_event,FAIL_REG_NET,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR ,RT_WAITING_FOREVER, RT_NULL);
		rt_kprintf("����NB�����쳣�����߳�\n");
//        rt_enter_critical();
		disconnect_nb_net();
		connect_nb_net();
		rt_kprintf("�˳�NB�����쳣�����߳�\n");
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
		rt_kprintf("����ģ���쳣�����߳�,ϵͳ��λ\r\n");
        HAL_NVIC_SystemReset();//system reset

		rt_thread_delay(1000 * 5);  //delay 5s
		
	}
	
}
void nb_thread_entry(void *parameter)
{
    cm_header_t ptr = &cm_default_config;	 //header
    uint16_t length;
//    rt_timer_init(&otatimer,"otatim",otatimer_callback,RT_NULL,1000 * 20,RT_TIMER_FLAG_ONE_SHOT);//��̬��ʱ�����г�ʼ��
	rt_event_init(&nb_event,"nb",RT_IPC_FLAG_FIFO); //��ʼ���¼���
	at_set_urc_table(urc_table,urc_table_size);
    rt_thread_delay(1000*5);
	kvinit();   //��ʼֵflash
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
        rt_kprintf("�����ϴ�ģ����Ϣ�߳�\n");
        cm_default_config.dev_type = CM_EARTH_DEV;     //�ر�
        cm_default_config.dev_id = CM_DBLF_ID;  
        config_data.cmd = cm_modulestatus_up;  
        length = cm_encode(ptr,&config_data);
        nb_send((char*)cm_senddata,length); 			
		rt_kprintf("�˳��ϴ�ģ����Ϣ�߳�\n");
//        rt_pm_request(PM_SLEEP_MODE_DEEP);    //����������˯��ģʽ
        rt_thread_delay(cmcycle); 
        
	}
}

void heart_thread_entry(void *parameter)
{
	cm_header_t ptr = &cm_default_config;
	uint16_t l;
	rt_event_recv(&nb_event,NBNET,RT_EVENT_FLAG_OR,RT_WAITING_FOREVER, RT_NULL);
    rt_thread_delay(1000 * 60 *2); //5�����ϴ�һ�������� cmcycleΪ�趨���ϱ�����
	while(1)
	{
		rt_kprintf("����heartbeat�߳�\n");
        refreshdata();
        config_data.cmd = cm_heartbeat_up;    //�ϴ�������
        cm_default_config.dev_type = CM_EARTH_DEV;     //�豸���͸�Ϊǽ�ѷ�����
        cm_default_config.dev_id = CM_DBLF_ID;
        l = cm_encode(ptr,&config_data);
//        while(module_error == 1);
        nb_send((char*)cm_senddata,l);  //�ϱ�������
		rt_kprintf("�˳�heartbeat�߳�\n");
//        rt_pm_request(PM_SLEEP_MODE_DEEP);    //����������˯��ģʽ
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
		rt_kprintf("����life�߳�\n");
//        while(module_error == 1);
		nb_send(life,3);          //�����豸
		rt_kprintf("�˳�life�߳�\n");
//        rt_pm_request(PM_SLEEP_MODE_DEEP);    //����������˯��ģʽ
	}
}
/* func: connect and open network */
int status;
//static void nb_opennetwork()
//{
//    at_response_t resp;     //resp:��Ӧ�ṹ��
//    resp = at_create_resp(512,3,500);
//	int status;
//    uint16_t n;
//    nb_hardware_reset();

//    rt_thread_delay(1000 * 7);
//	at_exec_cmd(resp,"AT+NATSPEED=9600,3,1,0,1,0,0");   // ���ò�����
////	at_exec_cmd(resp,"AT+NBAND=3");   //��ͨ
//	at_exec_cmd(resp,"AT+NBAND=8");   // �ƶ� ���Բ�����
////	at_exec_cmd(resp,"AT+NBAND=5");   // ����

//	if(at_client_wait_connect(1000*5) < 0)   // at_client_wait_connect():�ȴ�ģ���ʼ�����
//	{
//		rt_kprintf("����BC35-Gģ��ʧ��\n");
//	}
//    rt_thread_delay(1000 * 2);
//    do{
//		at_exec_cmd(resp,"AT+CFUN?");
//        rt_thread_delay(1000);	
//	}while(rt_event_recv(&nb_event,cfun,RT_EVENT_FLAG_OR,1000, RT_NULL) != RT_EOK);
//    do{
//		at_exec_cmd(resp,"AT+NBAND?");
//		at_resp_parse_line_args_by_kw(resp,"+NBAND:","+NBAND:%d",&status); //����ָ���ؼ����е���Ӧ����
//		rt_thread_delay(1000);
//	}while(status != 8);
//    at_exec_cmd(resp,"AT+CMEE=1");   //����������
//    at_exec_cmd(resp,"AT+CIMI");     //international mobile subscriber id
//    at_exec_cmd(resp,"AT+CGSN=1");     //��ȡNBIOT IMEI��Ϣ
//    at_exec_cmd(resp,"AT+CPSMS=0");  //disable the use of PSM
//    at_exec_cmd(resp,"AT+NCONFIG?");   //configure UE Behaviour
//    rt_thread_delay(1000 * 3);
//    at_exec_cmd(resp,"AT+COPS=1,2,\"46000\"");         //�ֶ�����
//	do{
//		at_exec_cmd(resp,"AT+CSQ");
//		at_resp_parse_line_args_by_kw(resp,"+CSQ:","+CSQ:%d",&status); //����ָ���ؼ����е���Ӧ����
//		rt_thread_delay(1000);
//	}while(status > 31);
//	
//	do{
//		at_exec_cmd(resp,"AT+CGATT?");
//		rt_thread_delay(1000);
//        }while(rt_event_recv(&nb_event,CGATT,RT_EVENT_FLAG_OR,1000, RT_NULL) != RT_EOK ); //RT_EVENT_FLAG_OR:�߼���ķ�ʽ�����¼�
//	
//	do{
//		at_exec_cmd(resp,"AT+CEREG?");
//		rt_thread_delay(1000);
//	}while(rt_event_recv(&nb_event,CEREG,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,1000, RT_NULL) != RT_EOK);

//    at_exec_cmd(resp,"AT+MIPLCONFIG=0,183.230.40.40,5683"); //configure LwM2M server IP and port
//    rt_thread_delay(1000);
//    at_exec_cmd(resp,"AT+MIPLCONFIG?");  //��ѯ  LwM2M server ip and port
//    rt_thread_delay(1000);
//    connect_nb_net();
//    at_delete_resp(resp);    
//}

/* query CSQ */
void bc35_query_csq(void)
{
    int status = 0;
    at_response_t resp;     //resp:��Ӧ�ṹ��
    resp = at_create_resp(512,3,500);
    
    do{
		at_exec_cmd(resp,"AT+CSQ");   //��ѯ�ź�ǿ��
        rt_thread_delay(300);	          //�����Ӧʱ��300ms
		at_resp_parse_line_args_by_kw(resp,"+CSQ:","+CSQ:%d",&status); //����ָ���ؼ����е���Ӧ����
		rt_thread_delay(1000);
	}while(status > 31);    //�ź�ǿ�ȴ���31,�ٴβ�ѯ��
    at_delete_resp(resp);
}
/* query UE'S IP */
void bc35_query_ip(void)
{
    uint8_t count = 0;
    at_response_t resp;     //resp:��Ӧ�ṹ��
    resp = at_create_resp(512,3,500);
    do{
        at_exec_cmd(resp,"AT+CGPADDR"); //��ѯģ���Ƿ��ȡ��IP
        rt_thread_delay(300);  //�����Ӧʱ��300ms
        if(at_resp_get_line_by_kw(resp,"OK") != NULL)
        {
            LOG_D("ģ���ѷ��䵽IP��ַ\r\n");
            break;
        }
        rt_thread_delay(1000);  //��ʱ1S�ٴβ�ѯ��ѭ��3�ν���
        count++;
    }while(count < 3);
    count = 0;  //��������
    at_delete_resp(resp);
}
/*����Ϊ���ŵ��Զ���������*/
static int nb_tel_autoconnect()
{
    int result = 0;
    uint8_t count = 0;     //resp:����ѭ������
	int status;
    rt_pin_write(NB_EN_PIN,PIN_HIGH);//��NBIOTģ���Դ��ģ���ϵ�
    rt_thread_delay(1000 * 5); //Լ5S������Neul OK��������ʾģ�����ִ��ATָ��
	if(at_client_wait_connect(1000*5) < 0)   // at_client_wait_connect():�ȴ�ģ���ʼ�����
	{
		LOG_D("����BC35-Gģ��ʧ��\n");
	}
    /* ���������� */
    check_send_cmd("AT+CMEE=1", AT_OK, 0, 300);
    if (result != RT_EOK) return result;
    /* ����UEΪ�Զ�����ģʽ */
    result = check_send_cmd("AT+NCONFIG=AUTOCONNECT,TRUE", AT_OK, 0, 300);
    if (result != RT_EOK) return result;
    /* ��λUE */
    check_send_cmd("AT+NRB", AT_OK, 0, 6000);
    while(RT_EOK != check_send_cmd("AT", AT_OK, 0, AT_DEFAULT_TIMEOUT))
    {
        rt_thread_mdelay(1000);
    }
    /* ���USIM���Ƿ��ʼ���ɹ� */
    do{
        result = check_send_cmd("AT+CIMI", "460", 0, 300);
        if (result == RT_EOK) 
        {
            LOG_D("��ȡ��USIM����IMSI��\r\n");
            break;
        }
        rt_thread_delay(1000);  //��ʱ1S�ٴβ�ѯ��ѭ��10�ν���
        count++;
       
    }while(count < 10);
    count = 0;  //��������
    
    /* ��ȡNBIOT IMEI��Ϣ */
    do{
        result = check_send_cmd("AT+CGSN=1", "+CGSN", 0, 300);
        if (result == RT_EOK) 
        {
            LOG_D("��ȡ��NBIOTģ��IMEI��Ϣ\r\n");
            break;
        }
        if(result != RT_EOK)
        {
            return result; 
        }
        rt_thread_delay(1000);  //��ʱ1S�ٴβ�ѯ��ѭ��10�ν���
        count++;
    }while(count < 3);
    count = 0;  //��������
    bc35_query_csq();
    /* ��ѯģ������總��״̬ */
    do{
		at_exec_cmd(resp,"AT+CEREG?"); //��ѯģ������總��״̬
		rt_thread_delay(1000);
	}while(rt_event_recv(&nb_event,CEREG,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,1000, RT_NULL) != RT_EOK);
    /* ��ѯģ���PDP�����ļ���״̬ */
    do{
		at_exec_cmd(resp,"AT+CGATT?");//��ѯģ���PDP�����ļ���״̬
		rt_thread_delay(1000);
    }while(rt_event_recv(&nb_event,CGATT,RT_EVENT_FLAG_OR,1000, RT_NULL) != RT_EOK ); //RT_EVENT_FLAG_OR:�߼���ķ�ʽ�����¼�
    /* query UE'S IP Address */
    bc35_query_ip();
    /* ����ָʾ����Ϣ */
    result = check_send_cmd("AT+NNMI=1", AT_OK, 0, 300);
    if (result != RT_EOK) return result;
    /* ��ģ�����ӵ�CDP������ʱ�������������ϱ���ǰע��״̬ */
    do{
		result = check_send_cmd("AT+NMSTATUS?", "+NMSTATUS", 0, 300); 
		rt_thread_delay(1000);
	}while(rt_event_recv(&nb_event,NMSTATUS,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,1000, RT_NULL) != RT_EOK);  
}
/*����Ϊ���ŵ��ֶ���������*/
static int nb_tel_manconnect()
{
    int result = 0;
    uint8_t count = 0;     //����ѭ������
	int status;
    rt_pin_write(NB_EN_PIN,PIN_HIGH);//��NBIOTģ���Դ��ģ���ϵ�
    rt_thread_delay(1000 * 5); //Լ5S������Neul OK��������ʾģ�����ִ��ATָ��
    if(at_client_wait_connect(1000*5) < 0)   // at_client_wait_connect():�ȴ�ģ���ʼ�����
	{
		LOG_D("����BC35-Gģ��ʧ��\n");
	}
    /* ���������� */
    check_send_cmd("AT+CMEE=1", AT_OK, 0, 300);
    if (result != RT_EOK) return result;
    /* ����UEΪ�ֶ�����ģʽ */
    result = check_send_cmd("AT+NCONFIG=AUTOCONNECT,FALSE", AT_OK, 0, 300);
    if (result != RT_EOK) return result;
    /* ����CDP������*/
    do{
        result = check_send_cmd("AT+NCDP=119.3.250.80,5683",AT_OK,0,300); //��Ϊ�Ƶ��Խӿ�iot-coaps.cn-north-4.myhuaweicloud.com
        if (result == RT_EOK) 
        {
            LOG_D("CDP���������óɹ�\r\n");
            break;
        }
        rt_thread_delay(1000);  //��ʱ1S�ٴβ�ѯ��ѭ��3�ν���
        count++;
    }while(count < 3);
    count = 0;  //��������
    /* ��λUE */
    check_send_cmd("AT+NRB", AT_OK, 0, 6000);
    while(RT_EOK != check_send_cmd("AT", AT_OK, 0, AT_DEFAULT_TIMEOUT))
    {
        rt_thread_mdelay(1000);
    }
    /* ����Ƶ���� */
    do{
        result = check_send_cmd("AT+CFUN=1", AT_OK, 0, 300);
        if (result == RT_EOK) 
        {
            LOG_D("��Ƶ�����Ѵ�\r\n");
            break;
        }
        rt_thread_delay(1000);  //��ʱ1S�ٴβ�ѯ��ѭ��5�ν���
        count++;
    }while(count < 5);
    count = 0;  //��������
    rt_thread_delay(5000);  //��ʱ5S�ٴβ�ѯ
    /* ���USIM���Ƿ��ʼ���ɹ� */
    do{
        result = check_send_cmd("AT+CIMI", "460", 0, 300);
        if (result == RT_EOK) 
        {
            LOG_D("��ȡ��USIM����IMSI��\r\n");
            break;
        }
        rt_thread_delay(1000);  //��ʱ1S�ٴβ�ѯ��ѭ��10�ν���
        count++;
    }while(count < 10);
    count = 0;  //��������
    /* ��ȡNBIOT IMEI��Ϣ */
    do{
        result = check_send_cmd("AT+CGSN=1", "+CGSN", 0, 300);
        if (result == RT_EOK) 
        {
            LOG_D("��ȡ��NBIOTģ��IMEI��Ϣ\r\n");
            break;
        }
        rt_thread_delay(1000);  //��ʱ1S�ٴβ�ѯ��ѭ��10�ν���
        count++;
    }while(count < 3);
    count = 0;  //��������
    /* PS���� */
    do{
        result = check_send_cmd("AT+CGATT=1", AT_OK, 0, 1000);
        if (result == RT_EOK) 
        {
            LOG_D("PS�������óɹ�\r\n");
            break;
        }
        rt_thread_delay(1000);  //��ʱ1S�ٴβ�ѯ��ѭ��3�ν���
        count++;
    }while(count < 3);
    count = 0;  //��������
    bc35_query_csq();
    do{
		at_exec_cmd(resp,"AT+CEREG?"); //��ѯģ������總��״̬
		rt_thread_delay(1000);
	}while(rt_event_recv(&nb_event,CEREG,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,1000, RT_NULL) != RT_EOK);
    do{
		at_exec_cmd(resp,"AT+CGATT?");//��ѯģ���PDP�����ļ���״̬
		rt_thread_delay(1000);
    }while(rt_event_recv(&nb_event,CGATT,RT_EVENT_FLAG_OR,1000, RT_NULL) != RT_EOK ); //RT_EVENT_FLAG_OR:�߼���ķ�ʽ�����¼�
    /* query UE'S IP Address */
    bc35_query_ip();
    /* ����ָʾ����Ϣ */
    result = check_send_cmd("AT+NNMI=1", AT_OK, 0, 300);
    if (result != RT_EOK) return result;
    /* ��ģ�����ӵ�CDP������ʱ�������������ϱ���ǰע��״̬ */
    do{
		result = check_send_cmd("AT+NMSTATUS?", "+NMSTATUS", 0, 300); 
		rt_thread_delay(1000);
	}while(rt_event_recv(&nb_event,NMSTATUS,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,1000, RT_NULL) != RT_EOK);  
}
uint32_t ackid = 1;
uint32_t ackid1 = 1;
/* �ƶ� */
//void nb_send(char* msg, uint16_t len)
//{
//    char send[512];
////    char cmdresult[5] = {0};
//    at_response_t resp;     //resp:��Ӧ�ṹ��
//    resp = at_create_resp(512,3,500);
//    Hex2Str(msg,send,len);
//    at_exec_cmd(resp,"AT+MIPLNOTIFY=0,%d,3200,0,5505,2,%d,%s,0,0,%d",obs_msgid0,len,send,ackid); //����ʵ��0 �ƶ�
//   
//    while(at_resp_parse_line_args(resp,2,"%s","OK") > 0)//�����ɹ�
//    {
//        rt_kprintf("�ϱ����ݳɹ�,��ɱ�������\n");
//        break;        
//    }
//    while(at_resp_parse_line_args(resp,1,"%s","ERROR") > 0)//�����ɹ�
//    {
//        rt_kprintf("�ϱ�����ʧ��,������������\n");
//        rt_event_recv(&nb_event,NBNET,RT_EVENT_FLAG_OR,RT_WAITING_FOREVER, RT_NULL); //��������ǰ����
//        at_exec_cmd(RT_NULL,"AT+MIPLUPDATE=0,86400,0");  //update lifetime
//        rt_kprintf("�����������ڳɹ�\n"); 
//        break;         
//    }
//    ackid++;
//    if(ackid > 65535)
//    {
//        ackid = 1;
//    }
//    at_delete_resp(resp);	
//}
/*��Ϊ��ƽ̨*/
void nb_send(char* msg, uint16_t len)
{
    char send[512];
    at_response_t resp;     //resp:��Ӧ�ṹ��
    resp = at_create_resp(512,3,500);
    Hex2Str(msg,send,len);
    at_exec_cmd(resp,"AT+NMGS=%d,%s",len,send); //�ϱ�����
   
    while(at_resp_parse_line_args(resp,2,"%s","OK") > 0)//�����ɹ�
    {
        rt_kprintf("�ϱ����ݳɹ�,��ɱ�������\n");
        break;        
    }
    at_delete_resp(resp);	
}
//void nb_send_ota(char* msg, uint16_t len)
//{
//    char send[512];
//    at_response_t resp;     //resp:��Ӧ�ṹ��
//    resp = at_create_resp(512,3,500);
//    Hex2Str(msg,send,len);
//    at_exec_cmd(RT_NULL,"AT+MIPLNOTIFY=0,%d,3200,1,5505,2,%d,%s,0,0,%d",obs_msgid1,len,send,ackid1); //����ʵ��1
//    ackid1++;
//    if(ackid1 > 65535)
//    {
//        ackid1 = 1;
//    }
//    while(at_resp_parse_line_args(resp,2,"%s","OK") > 0)//�����ɹ�
//    {
//        rt_kprintf("�ϱ����ݳɹ�,��ɱ�������\n");
//        break;
//    }
//    while(at_resp_parse_line_args(resp,2,"%s","ERROR") > 0)//�����ɹ�
//    {
//        rt_kprintf("�ϱ�����ʧ��,������������\n");
//        rt_event_recv(&nb_event,NBNET,RT_EVENT_FLAG_OR,RT_WAITING_FOREVER, RT_NULL); //��������ǰ����
//        at_exec_cmd(RT_NULL,"AT+MIPLUPDATE=0,86400,0");  //update lifetime
//        rt_kprintf("�����������ڳɹ�\n");
//        break;
//    }
//    at_delete_resp(resp);
//}

static void Hex2Str(char* pSrc, char* pDst, unsigned int nSrcLength)
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


void refreshdata(void)
{
	uint32_t rand;
	float randf;
	at_response_t resp;
	resp = at_create_resp(512,3,500);
	
	config_data.angle_x = deg_x;      //��������ȡ��X��ֵ
	config_data.angle_y = deg_y;      //��������ȡ��Y��ֵ
	
	config_data.dist = monitor_value[1];     //����
	config_data.bat_per = bat_q;           //����
	config_data.bat_vol = monitor_value[0];    //��ѹ
	
	config_data.gps_lat = deg_lat;      
	config_data.gps_long = deg_lon; 
	rand = rt_tick_get();
	randf =  (float)(rand%100)/1000000;
	config_data.bd_lat = deg_lat + randf; 
	rand = rt_tick_get();
	randf =  (float)((rand * rand)%100)/1000000;
	config_data.bd_long = deg_lon + randf;
    
    config_data.mud_n = (float)liqhigh;   //λ��ֵ��λmm
	config_data.rain_n = (float)(rainvalue * 0.1);  //��λmm
    config_data.auto_s = cmautos;   
	config_data.auto_lang  = cmautolang;  
                                       
	at_exec_cmd(resp,"AT+CSQ");
	at_resp_parse_line_args_by_kw(resp,"+CSQ:","+CSQ:%d",&csq);
	config_data.signal = csq;
	at_delete_resp(resp);
}

void connect_nb_net(void)
{
	at_response_t resp;     //resp:��Ӧ�ṹ��
	resp = at_create_resp(512,3,500);
	at_exec_cmd(resp,"AT+MIPLCREATE"); //�Խ��ƶ�ONEnetƽ̨����������ͨѶ�׼�ʵ��
	rt_thread_delay(1000 * 1);
	at_exec_cmd(resp,"AT+MIPLADDOBJ=0,3200,2,\"11\",1,1"); //��Ӷ����ʵ��,�Խ��ƶ�ONEnetƽ̨��ONENET�豸ʵ��ID:0,���3200(ISPO����������) ���Ͷ���2��ʵ��
	rt_thread_delay(1000 * 1);
	at_exec_cmd(resp,"AT+MIPLOPEN=0,86400,60");// ��¼ONENETƽ̨   lifetime = 86400s = 1day
	rt_thread_delay(1000 * 1);
//	if(rt_event_recv(&nb_event,EVENT,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, RT_NULL) == RT_EOK)
	if(rt_event_recv(&nb_event,EVENT,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,1000 * 60 * 15, RT_NULL) == RT_EOK) //15min
	{
			rt_kprintf("�����ɹ�\n");    
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
    rt_event_send(&nb_event,NBNET); //���������������¼�
	at_delete_resp(resp);

	
}
void disconnect_nb_net(void)
{
	at_response_t resp;     //resp:��Ӧ�ṹ��
	resp = at_create_resp(512,3,500);
	at_exec_cmd(resp,"AT+MIPLDELOBJ=0,3200"); //Delete a LWM2M object
	at_exec_cmd(resp,"AT+MIPLCLOSE=0"); 
	at_exec_cmd(resp,"AT+MIPLDELETE=0");
	at_delete_resp(resp);	
}

/*ˢ������*/
void refreshkvalue()
{
	uint32_t *raddr,waddr;
	raddr = (uint32_t*)&cm_config_kvalue;
	fal_read(0,raddr,sizeof(struct cm_configkv)/8);     //��flash KV��
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
		else                                                 //��ǰ�汾����֮ǰ�İ汾��
		{
			cm_soft_ver =(uint16_t) cm_config_kvalue.softversion;
		}
	}
	else
	{
		cm_config_kvalue.softversion = (uint32_t)cm_soft_ver;
	}
	
	waddr = (uint32_t)&cm_config_kvalue;
	fal_write(0,waddr,sizeof(struct cm_configkv)/8);          //дflahs  KV��
    rt_kprintf("flash ok\n");
}


static void kvinit()
{
	uint32_t *raddr;
	raddr = (uint32_t*)&cm_config_kvalue;
	fal_read(0,raddr,sizeof(struct cm_configkv)/4);     //��flash KV��; 
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
	if(cm_config_kvalue.softversion != 0xFFFFFFFF)    //������0XFFFFFFFF ˵��flash�а汾�Ѿ���д�롣
	{
		if(cm_config_kvalue.softversion < (uint32_t)cm_soft_ver)
		{
			cm_config_kvalue.softversion = (uint32_t)cm_soft_ver;
		}
		else                                                           //�������°汾
		{
            cm_soft_ver = (uint16_t)cm_config_kvalue.softversion;
		}
	}
	else                                                   //����0XFFFFFFFF ˵��flash�а汾δ��д�롣
	{
        cm_config_kvalue.softversion = (uint32_t)cm_soft_ver; //����ʼֵ�汾д��ȥ��
        
	}
}

static void firstrefreshkvalue()
{
    rt_kprintf("\r refresh kvalue ok!\r\n");
    uint32_t *raddr,waddr;
    raddr = (uint32_t*)&cm_config_kvalue;
    fal_read(0,raddr,sizeof(struct cm_configkv)/4);     //��flash KV��
  
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
    fal_write(0,waddr,sizeof(struct cm_configkv)/8);          //дflahs  KV��
}
MSH_CMD_EXPORT(firstrefreshkvalue, refresh first kvalue);