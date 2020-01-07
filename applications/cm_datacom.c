#include "cm_datacom.h"
#include <string.h>
#include <stdio.h>


#define VERSION                 0x01
uint32_t cmcycle = 1000 * 60 * 10; // 1h 转换成ms
uint32_t cmgpscycle = 1000 * 60 * 5;      //5min     //30min
uint32_t cmmotioncycle = 1000 * 60 * 5;   //20min 
uint16_t cmsample = 30; //单位s  //10s
float surface_angle = 12.0;     //地表角度阈值 float 4byte
float surface_dist = 10.0;      //地表拉线阈值 float 4byte
float wall_angle = 15.0;        //墙裂缝角度阈值 float 4byte
float wall_dist = 15.0;         //墙裂缝拉线阈值修改 float 4byte
float dmqx_angle = 15.0;        //地面倾斜角度阈值   float 4byte
float mudthreshold = 10.0;
uint16_t cmrain = 20;    //单位mm 阈值

uint8_t cmautos = 1,cmautolang = 0; //智能报警器参数默认设置值(关闭智能报警器电源,不播放语音)
extern float monitor_value[2];
extern uint8_t bat_q;
extern void connect_nb_net(void);
extern void disconnect_nb_net(void);
extern uint8_t sel_thread;
uint16_t cm_soft_ver = 0x1000;       //软件版本默认V1.000
uint8_t cm_hard_ver = 0x10;    // 硬件版本默认V1.0
uint8_t otaflag = 0;
uint32_t cm_packet_len = 0;
uint32_t cm_crc32 = 0;
uint32_t cm_rand_crc = 0;
uint32_t cm_packet_num = 0;
uint32_t cm_pack_index = 0;

struct rt_timer otatimer;
//static void cm_otadecodehandle(uint8_t *rev,uint16_t len);
//static void cm_otaencode_handle(cm_header_t head,cm_mainmsg_t msg,uint8_t *userdata);
extern void refreshkvalue();
//extern void nb_send_ota(char* msg, uint16_t len);
extern rt_thread_t tid1;
extern    rt_thread_t tid2;
extern	rt_thread_t tid3;
extern	rt_thread_t tid4;
    
 extern   rt_thread_t tid5;
extern	rt_thread_t tid6;
extern    rt_thread_t tid7;
#if USING_DBLH
struct cm_header cm_default_config = {
    VERSION,                  //version
	CM_EARTH_DEV,            //地表裂缝监测仪
    0x00,                   // spare byte
	cm_msg_up,              //上行
    CM_DBLF_ID,            // SN
    0x0017,                 //23个字节长度
	RT_NULL,
};
#endif
#if USING_QLH
struct cm_header cm_default_config = {
    VERSION,                  //version
	CM_WALL_DEV,            //地表裂缝监测仪
    0x00,                   // spare byte
	cm_msg_up,              //上行
    CM_QLF_ID,            // SN
    0x0017,                 //23个字节长度
	RT_NULL,
};
#endif
#if USING_DMQX
struct cm_header cm_default_config = {
    VERSION,                  //version
	CM_ANGLE_DEV,            //地表裂缝监测仪
    0x00,                   // spare byte
	cm_msg_up,              //上行
    CM_DMQX_ID,            // SN
    0x0017,                 //23个字节长度
	RT_NULL,
};
#endif
#if USING_MUD
struct cm_header cm_default_config = {
    VERSION,                  //version
	CM_MUD_DEV,            //地表裂缝监测仪
    0x00,                   // spare byte
	cm_msg_up,              //上行
    CM_MUD_ID,            // SN
    0x0017,                 //23个字节长度
	RT_NULL,
};
#endif
#if USING_RAIN
struct cm_header cm_default_config = {
    VERSION,                  //version
	CM_RAIN_DEV,            //地表裂缝监测仪
    0x00,                   // spare byte
	cm_msg_up,              //上行
    CM_RAIN_ID,            // SN
    0x0017,                 //23个字节长度
	RT_NULL,
};
#endif
#if USING_ALARMER
struct cm_header cm_default_config = {
    VERSION,                  //version
	CM_AUTO_DEV,            //地表裂缝监测仪
    0x00,                   // spare byte
	cm_msg_up,              //上行
    CM_ANNU_ID,            // SN
    0x0017,                 //23个字节长度
	RT_NULL,
};
#endif
struct cm_mainmsg config_data = {
		
        cm_heartbeat_up,       //上报心跳包
		"120619",
		"102015",
		0,     //方向位移
		0,
		0,
		0,       //倾角
		0,
		0,
		0,       //拉线
		0,            //电量百分比
	    12,        //电压值
		153.41,      //gps
	    29.588,
	    133.41,      //北斗
	    29.588,
		24,          //csq
	    1,          //雨量
	    1,           //泥位
        0,           //智能报警器电源开关 0：打开电源
	    0,           //智能报警器语音开关不播放语音
        0,           //ota
        0,           //ota
        0,           //ota
        0,           //ota
	};

extern void nb_send(char* msg,uint16_t len);

static uint16_t cm_surface_encode(cm_header_t head,cm_mainmsg_t msg);
static uint16_t cm_wall_encode(cm_header_t head,cm_mainmsg_t msg);
static uint16_t cm_angle_encode(cm_header_t head,cm_mainmsg_t msg);
static uint16_t cm_rain_encode(cm_header_t head,cm_mainmsg_t msg);
static uint16_t cm_mud_encode(cm_header_t head,cm_mainmsg_t msg);
static uint16_t cm_auto_encode(cm_header_t head,cm_mainmsg_t msg);
static void cm_init(cm_header_t head,uint8_t *dat,uint16_t len);

static void cm_earth_decode(uint8_t *rev,uint16_t len);
static void cm_wall_decode(uint8_t *rev,uint16_t len);
static void cm_angle_decode(uint8_t *rev,uint16_t len);
static void cm_rain_decode(uint8_t *rev,uint16_t len);
static void cm_mud_decode(uint8_t *rev,uint16_t len);
static void cm_auto_decode(uint8_t *rev,uint16_t len);
//static void cm_angle_decode(uint8_t *rev,uint16_t len);
/*按设备类型分别对数据编码*/
uint16_t cm_encode(cm_header_t head,cm_mainmsg_t msg)
{
	uint8_t lg=0;
	if(head->msg_mode==0x00 || head->msg_mode==0x01) //设备上行消息
	{
		head->user_data = msg;       
		if(head->dev_type == CM_EARTH_DEV)
		{
			head->dev_type = CM_EARTH_DEV;
			lg = cm_surface_encode(head,msg);
		}
        if(head->dev_type == CM_WALL_DEV)
		{
			head->dev_type = CM_WALL_DEV;
			lg = cm_wall_encode(head,msg);
		}
		else if(head->dev_type == CM_ANGLE_DEV)
		{
			head->dev_type = CM_ANGLE_DEV;
			lg = cm_angle_encode(head,msg);
		}
		else if(head->dev_type == CM_RAIN_DEV)
		{
			head->dev_type = CM_RAIN_DEV;
			lg = cm_rain_encode(head,msg);
		}
		else if(head->dev_type == CM_MUD_DEV)
		{
			head->dev_type = CM_MUD_DEV;
			lg = cm_mud_encode(head,msg);
		}
		else if(head->dev_type == CM_AUTO_DEV)
		{
			head->dev_type = CM_AUTO_DEV;
			lg = cm_auto_encode(head,msg);
		}
		
    }

    return (lg + 14); 
}

/*地表裂缝编码函数*/
static uint16_t cm_surface_encode(cm_header_t head,cm_mainmsg_t msg)
{
	uint8_t userdat[51] = {0};//字节长度需要计算 最大51字节
	float f;
	uint32_t d;
	uint32_t c;
    
    if(msg->cmd == cm_modulestatus_up)   //上报模块状态（地表/墙裂缝监测仪）
	
	{
			head->data_len = 51;         // 48 + 3 (包含1byte type(0xF6)  ,1byte datalen, n byte data 1 byte crc-8 )
            userdat[0] = 0xF6;  //固定，表示为NB监测类型
            userdat[1] = 0x2F;  //
            userdat[2] = cm_modulestatus_up;  // 01:上报地表/墙裂缝模块状态
			strcopy(&userdat[3],(uint8_t*)msg->date,7);
			strcopy(&userdat[10],(uint8_t*)msg->time,7);
			getfloat(&msg->dir_x,&userdat[17]);
			getfloat(&msg->dir_y,&userdat[21]);
			getfloat(&msg->dir_z,&userdat[25]);
			getfloat(&msg->angle_x,&userdat[29]);
			getfloat(&msg->angle_y,&userdat[33]);
			getfloat(&msg->angle_z,&userdat[37]);
			getfloat(&msg->dist,&userdat[41]);
			userdat[45] = msg->bat_per;
			getfloat(&msg->bat_vol,&userdat[46]);
		}
		else if(msg->cmd == cm_angle_up)     //上报角度偏移告警阈值（地表/墙裂缝监测仪）
		{
			f = surface_angle;   //地表角度偏移设定阈值
			head->data_len = 8;    // 5 + 3 + 14 = 22
            userdat[0] = 0xF6;  //固定，表示为NB监测类型
            userdat[1] = 0x05;  
            userdat[2] = cm_angle_up;  //上报角度偏移告警阈值（地表/墙裂缝监测仪）
			getfloat(&f,&userdat[3]);
		}
		else if(msg->cmd == cm_dist_up)      //上报拉线偏移告警阈值（地表/墙裂缝监测仪）
		{
			f = surface_dist;     //地表拉线设定阈值
            head->data_len = 8;    // 5 + 3 + 14 = 22
            userdat[0] = 0xF6;  //固定，表示为NB监测类型
            userdat[1] = 0x05;  //
            userdat[2] = cm_dist_up;  //上报拉线偏移告警阈值（地表/墙裂缝监测仪）
			
			getfloat(&f,&userdat[3]);
		}
		else if(msg->cmd == cm_cycle_up)        //上报数据周期（地表/墙裂缝监测仪）
		{
			c = cmcycle;
			head->data_len = 16;        //13 + 3 +14 = 30
            userdat[0] = 0xF6;  //固定，表示为NB监测类型
            userdat[1] = 0x0D;  //
            userdat[2] = cm_cycle_up;  //上报数据周期（地表/墙裂缝监测仪）
            userdat[3] = cmgpscycle & 0x000000FF;
            userdat[4] = cmgpscycle>>8 & 0x000000FF;
            userdat[5] = cmgpscycle>>16 & 0x000000FF;
            userdat[6] = cmgpscycle>>24 & 0x000000FF;
            
            userdat[7] = cmmotioncycle & 0x000000FF;;
            userdat[8] = cmmotioncycle>>8 & 0x000000FF;
            userdat[9] = cmmotioncycle>>16 & 0x000000FF;
            userdat[10] = cmmotioncycle>>24 & 0x000000FF;
            
            userdat[11] = cmcycle & 0x000000FF;;
            userdat[12] = cmcycle>>8 & 0x000000FF;
            userdat[13] = cmcycle>>16 & 0x000000FF;
            userdat[14] = cmcycle>>24 & 0x000000FF;
		
		}
		else if(msg->cmd == cm_softversion_up )      //  上报软件版本 （地表/墙裂缝监测仪）
		{   
			d = 0x101000;   //1000表示软件版本V1.000 10表示硬件版本V1.0
			head->data_len = 7;      //4 + 3 + 14 = 21
            userdat[0] = 0xF6;  //固定，表示为NB监测类型
            userdat[1] = 0x04;  //
            userdat[2] = cm_softversion_up;  //  上报软件版本 （地表/墙裂缝监测仪）
			userdat[3] = (cm_soft_ver>>8) & 0x00FF;
			userdat[4] = cm_soft_ver & 0x00FF;
			userdat[5] = cm_hard_ver;  //硬件版本
		}
		else if(msg->cmd == cm_sample_up)       //  上报采样间隔 （地表/墙裂缝监测仪）
		{
			d = cmsample;   //2400(0x0960)      2400S 小端模式 0x00 0x60 0x0960
			head->data_len = 7;  //4 + 3 + 14 = 21
			userdat[0] = 0xF6;
            userdat[1] = 0x04;  //
            userdat[2] = cm_sample_up;  //  上报采样间隔 （地表/墙裂缝监测仪）
			userdat[3] = 0;   //0代表秒
			userdat[4] = d & 0x00ff;
            userdat[5]= d>>8 & 0xff;
		}
		else if(msg->cmd == cm_heartbeat_up)        //  上报心跳包 （地表/墙裂缝监测仪）
		{
			head->data_len = 23;             //20 + 3 + 14 = 37
            userdat[0] = 0xF6;
            userdat[1] = 0x14;  //
            userdat[2] = cm_heartbeat_up;  //  上报心跳包 （地表/墙裂缝监测仪）
			getfloat(&msg->gps_lat,&userdat[3]);
			getfloat(&msg->gps_long,&userdat[7]);
			getfloat(&msg->bd_lat,&userdat[11]);
			getfloat(&msg->bd_long,&userdat[15]);
			userdat[19] = msg->bat_per;
			userdat[20] = msg->signal & 0xff;
			userdat[21] = msg->signal >>8 & 0xff;
			
		}
//        cm_otaencode_handle(head,msg,userdat);  
		cm_init(head,userdat,head->data_len);
        return 	head->data_len;  
}

/*墙裂缝编码函数*/
static uint16_t cm_wall_encode(cm_header_t head,cm_mainmsg_t msg)
{
	uint8_t userdat[51] = {0};//字节长度需要计算 最大51字节
	float f;
	uint32_t d;
	uint32_t c;
    
    if(msg->cmd == cm_modulestatus_up)   //上报模块状态（地表/墙裂缝监测仪）
	
	{
			head->data_len = 51;         // 48 + 3 (包含1byte type(0xF6)  ,1byte datalen, n byte data 1 byte crc-8 )
            userdat[0] = 0xF6;  //固定，表示为NB监测类型
            userdat[1] = 0x2F;  //
            userdat[2] = cm_modulestatus_up;  // 01:上报地表/墙裂缝模块状态
			strcopy(&userdat[3],(uint8_t*)msg->date,7);
			strcopy(&userdat[10],(uint8_t*)msg->time,7);
			getfloat(&msg->dir_x,&userdat[17]);
			getfloat(&msg->dir_y,&userdat[21]);
			getfloat(&msg->dir_z,&userdat[25]);
			getfloat(&msg->angle_x,&userdat[29]);
			getfloat(&msg->angle_y,&userdat[33]);
			getfloat(&msg->angle_z,&userdat[37]);
			getfloat(&msg->dist,&userdat[41]);
			userdat[45] = msg->bat_per;
			getfloat(&msg->bat_vol,&userdat[46]);
		}
		else if(msg->cmd == cm_angle_up)     //上报角度偏移告警阈值（墙裂缝监测仪）
		{
			f = wall_angle;   //墙角度偏移设定阈值
			head->data_len = 8;    // 5 + 3 + 14 = 22
            userdat[0] = 0xF6;  //固定，表示为NB监测类型
            userdat[1] = 0x05;  
            userdat[2] = cm_angle_up;  //上报角度偏移告警阈值（墙裂缝监测仪）
			getfloat(&f,&userdat[3]);
		}
		else if(msg->cmd == cm_dist_up)      //上报拉线偏移告警阈值（墙裂缝监测仪）
		{
			f = wall_dist;     //墙拉线设定阈值
            head->data_len = 8;    // 5 + 3 + 14 = 22
            userdat[0] = 0xF6;  //固定，表示为NB监测类型
            userdat[1] = 0x05;  //
            userdat[2] = cm_dist_up;  //上报拉线偏移告警阈值（地表/墙裂缝监测仪）
			
			getfloat(&f,&userdat[3]);
		}
		else if(msg->cmd == cm_cycle_up)        //上报数据周期（地表/墙裂缝监测仪）
		{
			c = cmcycle;
			head->data_len = 16;        //13 + 3 +14 = 30
            userdat[0] = 0xF6;  //固定，表示为NB监测类型
            userdat[1] = 0x0D;  //
            userdat[2] = cm_cycle_up;  //上报数据周期（地表/墙裂缝监测仪）
            userdat[3] = cmgpscycle & 0x000000FF;
            userdat[4] = cmgpscycle>>8 & 0x000000FF;
            userdat[5] = cmgpscycle>>16 & 0x000000FF;
            userdat[6] = cmgpscycle>>24 & 0x000000FF;
            
            userdat[7] = cmmotioncycle & 0x000000FF;;
            userdat[8] = cmmotioncycle>>8 & 0x000000FF;
            userdat[9] = cmmotioncycle>>16 & 0x000000FF;
            userdat[10] = cmmotioncycle>>24 & 0x000000FF;
            
            userdat[11] = cmcycle & 0x000000FF;;
            userdat[12] = cmcycle>>8 & 0x000000FF;
            userdat[13] = cmcycle>>16 & 0x000000FF;
            userdat[14] = cmcycle>>24 & 0x000000FF;
		
		}
		else if(msg->cmd == cm_softversion_up )      //  上报软件版本 （地表/墙裂缝监测仪）
		{   
			d = 0x101000;   //1000表示软件版本V1.000 10表示硬件版本V1.0
			head->data_len = 7;      //4 + 3 + 14 = 21
            userdat[0] = 0xF6;  //固定，表示为NB监测类型
            userdat[1] = 0x04;  //
            userdat[2] = cm_softversion_up;  //  上报软件版本 （地表/墙裂缝监测仪）
			userdat[3] = (cm_soft_ver>>8) & 0x00FF;
			userdat[4] = cm_soft_ver & 0x00FF;
			userdat[5] = cm_hard_ver;  //硬件版本
		}
		else if(msg->cmd == cm_sample_up)       //  上报采样间隔 （地表/墙裂缝监测仪）
		{
			d = cmsample;   //2400(0x0960)      2400S 小端模式 0x00 0x60 0x0960
			head->data_len = 7;  //4 + 3 + 14 = 21
			userdat[0] = 0xF6;
            userdat[1] = 0x04;  //
            userdat[2] = cm_sample_up;  //  上报采样间隔 （地表/墙裂缝监测仪）
			userdat[3] = 0;   //0代表秒
			userdat[4] = d & 0x00ff;
            userdat[5]= d>>8 & 0xff;
		}
		else if(msg->cmd == cm_heartbeat_up)        //  上报心跳包 （地表/墙裂缝监测仪）
		{
			head->data_len=23;             //20 + 3 + 14 = 37
            userdat[0] = 0xF6;
            userdat[1] = 0x14;  //
            userdat[2] = cm_heartbeat_up;  //  上报心跳包 （地表/墙裂缝监测仪）
			getfloat(&msg->gps_lat,&userdat[3]);
			getfloat(&msg->gps_long,&userdat[7]);
			getfloat(&msg->bd_lat,&userdat[11]);
			getfloat(&msg->bd_long,&userdat[15]);
			userdat[19] = msg->bat_per;
			userdat[20] = msg->signal & 0xff;
			userdat[21] = msg->signal >>8 & 0xff;
			
		}
//        cm_otaencode_handle(head,msg,userdat);  
		cm_init(head,userdat,head->data_len);
        return 	head->data_len;  
}

/*地面倾斜倾角编码函数*/
static uint16_t cm_angle_encode(cm_header_t head,cm_mainmsg_t msg)
{
	uint8_t userdat[49]={0};  //字节长度需要计算
	float f;
	uint32_t d;
	uint32_t c;
    
	if(msg->cmd == cm_modulestatus_up)        //上报模块状态（地面倾斜监测仪）
	{
        head->data_len = 35; //info长度31 、cmd 、0xf6和CRC、datalen 各一字节
        userdat[0] = 0xF6;  //固定，表示为NB监测类型
        userdat[1] = 0x20;  //info和cmd长度
        userdat[2] = cm_modulestatus_up;  // 01:上报地面倾斜监测仪模块状态
        strcopy(&userdat[3],(uint8_t*)msg->date,7);    //data
        strcopy(&userdat[10],(uint8_t*)msg->time,7);   //time
        getfloat(&msg->angle_x,&userdat[17]);
        getfloat(&msg->angle_y,&userdat[21]);
        getfloat(&msg->angle_z,&userdat[25]);
        
        userdat[29]=msg->bat_per;
        getfloat(&msg->bat_vol,&userdat[30]);
    }	
    else if(msg->cmd == cm_angle_up)     //上报角度偏移告警阈值（地面倾斜监测仪）
    {
        f = dmqx_angle;            //浮点型,来自平台设定的阈值
        head->data_len = 8;
        userdat[0] = 0xF6;  //固定，表示为NB监测类型
        userdat[1] = 0x05;  
        userdat[2] = cm_angle_up;  //上报角度偏移告警阈值（地面倾斜监测仪）
        getfloat(&f,&userdat[3]);
        
    }
    else if(msg->cmd == cm_cycle_up)       //上报数据周期（地面倾斜监测仪）
    {
        c = cmcycle;
        head->data_len = 16;
        userdat[0] = 0xF6;  //固定，表示为NB监测类型
        userdat[1] = 0x0D;  //
        userdat[2] = cm_cycle_up;  //上报数据周期（地面倾斜监测仪）
        userdat[3] = cmgpscycle & 0x000000FF;
        userdat[4] = cmgpscycle>>8 & 0x000000FF;
        userdat[5] = cmgpscycle>>16 & 0x000000FF;
        userdat[6] = cmgpscycle>>24 & 0x000000FF;
        
        userdat[7] = cmmotioncycle & 0x000000FF;;
        userdat[8] = cmmotioncycle>>8 & 0x000000FF;
        userdat[9] = cmmotioncycle>>16 & 0x000000FF;
        userdat[10] = cmmotioncycle>>24 & 0x000000FF;
        
        userdat[11] = cmcycle & 0x000000FF;;
        userdat[12] = cmcycle>>8 & 0x000000FF;
        userdat[13] = cmcycle>>16 & 0x000000FF;
        userdat[14] = cmcycle>>24 & 0x000000FF;

        
    }
    else if(msg->cmd == cm_softversion_up)      //  上报软件版本 （地面倾斜监测仪）
    {   
//        d = 0x101000;   //1000表示软件版本V1.000 10表示硬件版本V1.0
        head->data_len = 7;
        userdat[0] = 0xF6;  //固定，表示为NB监测类型
        userdat[1] = 0x04;  //
        userdat[2] = cm_softversion_up;  //  上报软件版本 （地面倾斜监测仪）
        userdat[3] = (cm_soft_ver>>8) & 0x00FF;
        userdat[4] = cm_soft_ver & 0x00FF;
        userdat[5]= cm_hard_ver;  //硬件版本
       
    }
    else if(msg->cmd == cm_sample_up)       //  上报采样间隔（地面倾斜监测仪）
    {
        d = cmsample;   //2400(0x0960)      2400S 小端模式 0x00 0x60 0x0960
        head->data_len = 7;
        userdat[0]=0xF6;
        userdat[1] = 0x04;  //
        userdat[2] = cm_sample_up;  //  上报采样间隔 （地表/墙裂缝监测仪）
        userdat[3]= 0;   //0代表秒
        userdat[4]= d & 0x00ff;
        userdat[5]= d>>8 & 0xff;
        
    }
    else if(msg->cmd == cm_heartbeat_up)        //上报心跳包 （地表/墙裂缝监测仪）
    {
        head->data_len = 23;
        userdat[0] = 0xF6;
        userdat[1] = 0x14;  //
        userdat[2] = cm_heartbeat_up;  //  上报心跳包 （地表/墙裂缝监测仪）
        getfloat(&msg->gps_lat,&userdat[3]);
        getfloat(&msg->gps_long,&userdat[7]);
        getfloat(&msg->bd_lat,&userdat[11]);
        getfloat(&msg->bd_long,&userdat[15]);
        userdat[19]=msg->bat_per;
        userdat[20]=msg->signal & 0xff;
        userdat[21]=msg->signal >>8 & 0xff;
        
        
    }
//		head->cmd |= head->dev_type<<8;
//    cm_otaencode_handle(head,msg,userdat);    
    cm_init(head,userdat,head->data_len);
 	return 	head->data_len;  
}

/*雨量编码函数*/
static uint16_t cm_rain_encode(cm_header_t head,cm_mainmsg_t msg)
{
	uint8_t userdat[37]= {0};
	float f;
	uint32_t d;
	uint32_t c;
//    uint32_t gps_up_cycle = 3000;            //GPS采样周期,4byte   3000ms = 3s = 0.05min
//    uint32_t motion_up_cycle = 0;         //运动上报周期,4byte
//    uint32_t up_cycle = 5*1000*60 ;                //上传间隔,4byte    5min
	
    if(msg->cmd == cm_modulestatus_up)      //主动上报消息
    {
        head->data_len = 13;
        userdat[0] = 0xF6;  //固定，表示为NB监测类型
        userdat[1] = 0x0A;  //
        userdat[2] = cm_modulestatus_up;  // 01:上报雨量计模块状态
        getfloat(&msg->rain_n,&userdat[3]);
        userdat[7]=msg->bat_per;
        getfloat(&msg->bat_vol,&userdat[8]);
    }
    else if(msg->cmd == cm_rain_threshold_up)     //上报告警阈值 (雨量) 2个字节
    {
        uint16_t rain_level;
        rain_level = cmrain ;
        rain_level = rain_level * 10;//数据扩大10倍传输
//        rain_level = (uint16_t)f;
        head->data_len = 6;
        userdat[0] = 0xF6;  //固定，表示为NB监测类型
        userdat[1] = 0x03;  //
        userdat[2] = cm_rain_threshold_up;  //告警阈值 (雨量)
        userdat[3] = rain_level & 0xff;
        userdat[4] = rain_level>>8 & 0xff;  
//        getfloat(&f,&userdat[1]);
    }
    else if(msg->cmd == cm_rain_cycle_up)       //上报数据周期（雨量计）
    {
        c = cmcycle;
        head->data_len = 16;
        userdat[0] = 0xF6;  //固定，表示为NB监测类型
        userdat[1] = 0x0D;  //
        userdat[2] = cm_rain_cycle_up;  //上报数据周期（地表/墙裂缝监测仪）
        userdat[3] = cmgpscycle & 0x000000FF;
        userdat[4] = cmgpscycle>>8 & 0x000000FF;
        userdat[5] = cmgpscycle>>16 & 0x000000FF;
        userdat[6] = cmgpscycle>>24 & 0x000000FF;
        
        userdat[7] = cmmotioncycle & 0x000000FF;;
        userdat[8] = cmmotioncycle>>8 & 0x000000FF;
        userdat[9] = cmmotioncycle>>16 & 0x000000FF;
        userdat[10] = cmmotioncycle>>24 & 0x000000FF;
        
        userdat[11] = cmcycle & 0x000000FF;;
        userdat[12] = cmcycle>>8 & 0x000000FF;
        userdat[13] = cmcycle>>16 & 0x000000FF;
        userdat[14] = cmcycle>>24 & 0x000000FF;
        
        
    }
    else if(msg->cmd == cm_rain_softversion_up)      //上报软件版本 
    {   
//        d = 0x101000;   //1000表示软件版本V1.000 10表示硬件版本V1.0
        head->data_len = 7;
        userdat[0] = 0xF6;  //固定，表示为NB监测类型
        userdat[1] = 0x04;  //
        userdat[2] = cm_rain_softversion_up;  //  上报软件版本 （地表/墙裂缝监测仪）
        userdat[3] = (cm_soft_ver>>8) & 0x00FF;
        userdat[4]= cm_soft_ver & 0x00FF;
        
        userdat[5]= cm_hard_ver;  //硬件版本
        
    }
    else if(msg->cmd == cm_rain_sample_up)       //采样频率
    {
        d = cmsample;   //2400(0x0960)      2400S 小端模式 0x00 0x60 0x0960
        head->data_len = 7;  //4 + 3 + 14 = 21
        userdat[0]=0xF6;
        userdat[1] = 0x04;  //
        userdat[2] = cm_rain_sample_up;  //  上报采样间隔 （地表/墙裂缝监测仪）
        userdat[3]= 0;   //0代表秒
        userdat[4]= d & 0x00ff;
        userdat[5]= d>>8 & 0xff;
        
    }
    else if(msg->cmd == cm_heartbeat_up)        //心跳
    {
        head->data_len = 23;             //20 + 3 + 14 = 37
        userdat[0] = 0xF6;
        userdat[1] = 0x14;  //
        userdat[2] = cm_heartbeat_up;  //  上报心跳包 
        getfloat(&msg->gps_lat,&userdat[3]);
        getfloat(&msg->gps_long,&userdat[7]);
        getfloat(&msg->bd_lat,&userdat[11]);
        getfloat(&msg->bd_long,&userdat[15]);
        userdat[19]= msg->bat_per;
        userdat[20]= msg->signal & 0xff;
        userdat[21]= msg->signal >>8 & 0xff;
        
    }
//    cm_otaencode_handle(head,msg,userdat);
//    head->cmd |= head->dev_type<<8; 
    cm_init(head,userdat,head->data_len);
	return 	head->data_len;  
}

/*泥位编码函数*/
static uint16_t cm_mud_encode(cm_header_t head,cm_mainmsg_t msg)
{
	uint8_t userdat[37]={0};
	float f;
	uint32_t d;
    if(msg->cmd == cm_modulestatus_up)      //主动上报消息
    {
        head->data_len = 13;
        userdat[0] = 0xF6;  //固定，表示为NB监测类型
        userdat[1] = 0x07;  //我认为甲方给定的长度为错误的
        userdat[2] = cm_modulestatus_up;  // 01:上报模块状态
        getfloat(&msg->mud_n,&userdat[3]);
        userdat[7]=msg->bat_per;
        getfloat(&msg->bat_vol,&userdat[8]);
    }
    else if(msg->cmd == cm_mud_threshold_up)    // 上报泥位告警阈值
    {
        
        f = mudthreshold;
        head->data_len = 8;
        userdat[0] = 0xF6;
        userdat[1] = 0x05;
        userdat[2] = cm_mud_threshold_up;  // 01:上报地表/墙裂缝模块状态
        getfloat(&f,&userdat[3]);
    }
    else if(msg->cmd == cm_cycle_up)       //周期  单位ms
    {
//        c = cmcycle;
        head->data_len = 16;
        userdat[0] = 0xF6;  //固定，表示为NB监测类型
        userdat[1] = 0x0D;  //
        userdat[2] = cm_cycle_up;  //上报数据周期（地表/墙裂缝监测仪）
        userdat[3] = cmgpscycle & 0x000000FF;
        userdat[4] = cmgpscycle>>8 & 0x000000FF;
        userdat[5] = cmgpscycle>>16 & 0x000000FF;
        userdat[6] = cmgpscycle>>24 & 0x000000FF;
        
        userdat[7] = cmmotioncycle & 0x000000FF;;
        userdat[8] = cmmotioncycle>>8 & 0x000000FF;
        userdat[9] = cmmotioncycle>>16 & 0x000000FF;
        userdat[10] = cmmotioncycle>>24 & 0x000000FF;
        
        userdat[11] = cmcycle & 0x000000FF;;
        userdat[12] = cmcycle>>8 & 0x000000FF;
        userdat[13] = cmcycle>>16 & 0x000000FF;
        userdat[14] = cmcycle>>24 & 0x000000FF;

        
    }
    else if(msg->cmd == cm_softversion_up)      //上报泥位软件版本
    {   
//        d = 0x101000;   //1000表示软件版本V1.000 10表示硬件版本V1.0
        head->data_len = 7;
        userdat[0] = 0xF6;  //固定，表示为NB监测类型
        userdat[1] = 0x04;  //
        userdat[2] = cm_softversion_up;  //  上报软件版本 （地表/墙裂缝监测仪）
        
        userdat[3] = (cm_soft_ver>>8) & 0x00FF;
        userdat[4]= cm_soft_ver & 0x00FF;
        
        userdat[5]= cm_hard_ver;  //硬件版本 //硬件版本
        
    }
    else if(msg->cmd == cm_sample_up)       //  上报采样间隔 （地表/墙裂缝监测仪）
    {
        d = cmsample;    //2400(0x0960)      2400S 小端模式 0x00 0x60 0x0960
        head->data_len = 7;
        userdat[0]=0xF6;
        userdat[1] = 0x04;  //
        userdat[2] = cm_sample_up;  //  上报采样间隔 （地表/墙裂缝监测仪）
        userdat[3]= 0;   //0代表秒
        userdat[4]= d & 0x00ff;
        userdat[5]= d>>8 & 0xff;
    
    }
    else if(msg->cmd == cm_heartbeat_up)        //心跳
    {
        head->data_len = 23;
        userdat[0] = 0xF6;
        userdat[1] = 0x14;  //
        userdat[2] = cm_heartbeat_up;  //  上报心跳包 （地表/墙裂缝监测仪）
        getfloat(&msg->gps_lat,&userdat[3]);
        getfloat(&msg->gps_long,&userdat[7]);
        getfloat(&msg->bd_lat,&userdat[11]);
        getfloat(&msg->bd_long,&userdat[15]);
        userdat[19]=msg->bat_per;
        userdat[20]=msg->signal & 0xff;
        userdat[21]=msg->signal >>8 & 0xff;
        
    }
//    cm_otaencode_handle(head,msg,userdat);
    cm_init(head,userdat,head->data_len);
	return 	head->data_len;  
}

/*智能设备编码函数*/
static uint16_t cm_auto_encode(cm_header_t head,cm_mainmsg_t msg)
{
	uint8_t userdat[37]={0};
	uint32_t d;
	uint32_t c;
//    uint32_t gps_up_cycle = 3000;            //GPS采样周期,4byte   3000ms = 3s = 0.05min
//    uint32_t motion_up_cycle = 0;         //运动上报周期,4byte
//    uint32_t up_cycle = 5*1000*60 ;                //上传间隔,4byte    5min
	
    if(msg->cmd == cm_modulestatus_up)      //主动上报消息
    {
        head->data_len = 6;
        userdat[0] = 0xF6;  //固定，表示为NB监测类型
        userdat[1] = 0x03;  //
        userdat[2] = cm_modulestatus_up;  // 01:上报状态
        userdat[3] = msg->auto_s;
        userdat[4] = msg->auto_lang;
    }
    else if(msg->cmd == cm_annu_up)     //上报智能报警器参数（智能报警器）
    {
        head->data_len = 6;
        userdat[0] = 0xF6;  //固定，表示为NB监测类型
        userdat[1] = 0x03;  //
        userdat[2] = cm_annu_up;  // 01:上报状态
       
        userdat[3] = msg->auto_s;
        userdat[4] = msg->auto_lang;
    }
    else if(msg->cmd == cm_annu_cycle_up)       //上报数据周期单位ms
    {
        c = cmcycle;
        head->data_len = 16;        //13 + 3 +14 = 30
        userdat[0] = 0xF6;  //固定，表示为NB监测类型
        userdat[1] = 0x0D;  //
        userdat[2] = cm_annu_cycle_up;  //上报数据周期（地表/墙裂缝监测仪）
        userdat[3] = cmgpscycle & 0x000000FF;
        userdat[4] = cmgpscycle>>8 & 0x000000FF;
        userdat[5] = cmgpscycle>>16 & 0x000000FF;
        userdat[6] = cmgpscycle>>24 & 0x000000FF;
        
        userdat[7] = cmmotioncycle & 0x000000FF;;
        userdat[8] = cmmotioncycle>>8 & 0x000000FF;
        userdat[9] = cmmotioncycle>>16 & 0x000000FF;
        userdat[10] = cmmotioncycle>>24 & 0x000000FF;
        
        userdat[11] = cmcycle & 0x000000FF;;
        userdat[12] = cmcycle>>8 & 0x000000FF;
        userdat[13] = cmcycle>>16 & 0x000000FF;
        userdat[14] = cmcycle>>24 & 0x000000FF;
  
        
    }
    else if(msg->cmd == cm_annu_softversion_up)      //版本
    {   
//        d = 0x101000;   //1000表示软件版本V1.000 10表示硬件版本V1.0
        head->data_len = 7;      //4 + 3 + 14 = 21
        userdat[0] = 0xF6;  //固定，表示为NB监测类型
        userdat[1] = 0x04;  //
        userdat[2] = cm_annu_softversion_up;  //  上报软件版本 （地表/墙裂缝监测仪）
        userdat[3] = (cm_soft_ver>>8) & 0x00FF;
        userdat[4]= cm_soft_ver & 0x00FF;
        
        userdat[5]= cm_hard_ver;  //硬件版本
       
    }
    else if(msg->cmd == cm_annu_sample_up)       //上报采样间隔 (智能报警器)
    {
        d = cmsample;   //2400(0x0960)      2400S 小端模式 0x00 0x60 0x0960
        head->data_len = 7;  //4 + 3 + 14 = 21
        userdat[0] = 0xF6;
        userdat[1] = 0x04;  //
        userdat[2] = cm_annu_sample_up;  //  上报采样间隔 （地表/墙裂缝监测仪）
        userdat[3]= 0;   //0代表秒
        userdat[4]= d & 0x00ff;
        userdat[5]= d>>8 & 0xff;
    }
    else if(msg->cmd == cm_heartbeat_up)        //心跳
    {
        head->data_len = 23;
        userdat[0] = 0xF6;
        userdat[1] = 0x14;  //
        userdat[2] = cm_heartbeat_up;  //
        
        getfloat(&msg->gps_lat,&userdat[3]);
        getfloat(&msg->gps_long,&userdat[7]);
        getfloat(&msg->bd_lat,&userdat[11]);
        getfloat(&msg->bd_long,&userdat[15]);
        userdat[19] = msg->bat_per;
        userdat[20] = msg->signal & 0xff;
        userdat[21] = msg->signal >>8 & 0xff;
        
    }
    else if(msg->cmd == cm_client_request_otadata)        //设备向服务器请求升级数据
    {
        head->data_len = 12;
        userdat[0] = 0xF6;
        userdat[1] = 0x09;  //
        userdat[2] = cm_client_request_otadata;  //  0xF2
        
        userdat[3] = msg->pack_index & 0x000000FF;                  
        userdat[4] = msg->pack_index>>8 & 0x000000FF;
        userdat[5] = msg->pack_index>>16 & 0x000000FF;
        userdat[6] = msg->pack_index>>24 & 0x000000FF; //index会变（msg->pack_index++）
        
        userdat[7] = msg->pack_num & 0x000000FF;             
        userdat[8] = msg->pack_num>>8 & 0x000000FF;
        userdat[9] = msg->pack_num>>16 & 0x000000FF;
        userdat[10] = msg->pack_num>>24 & 0x000000FF;    // 当前请求分包个数，分包个数一直为1,每次请求一个包。
   
    }
    else if(msg->cmd == cm_ota_status_up)        // 设备上报远程升级状态
    {
        head->data_len = 9;
        userdat[0] = 0xF6;
        userdat[1] = 0x06;  //
        userdat[2] = cm_ota_status_up;  //  0xF0
        userdat[3] = msg->ota_lab;     // ota_lab ：1；设备放弃本次升级，升级失败；2：本次升级数据包传输成功 
        userdat[4] = msg->ota_sum & 0x000000FF;      //ota_sum
        userdat[5] = msg->ota_sum>>8 & 0x000000FF;             
        userdat[6] = msg->ota_sum>>16 & 0x000000FF;
        userdat[7] = msg->ota_sum>>24 & 0x000000FF; //cm_rand_crc:分包码累加和    
    }
//    cm_otaencode_handle(head,msg,userdat);
//    head->cmd |= head->dev_type<<8; 
    cm_init(head,userdat,head->data_len);
	return 	head->data_len;  
}

static void cm_init(cm_header_t head,uint8_t *dat,uint16_t len)       //组装消息 len = 第二层的数据结构的长度 dat:第二层数据结构
{
	memset(cm_senddata,0,512);
	cm_senddata[cm_head_que] = head->version;
    cm_senddata[cm_count_que] = head->dev_type;
    cm_senddata[cm_spare_byte] = head->spare;
	cm_senddata[cm_data_type_que] = head->msg_mode;
	
	strcopy(&cm_senddata[cm_sn_que],(uint8_t*)head->dev_id,8);
	
	cm_senddata[cm_datalen1_que] = head->data_len & 0x00ff;            //2字节的数据长度
	cm_senddata[cm_datalen1_que + 1] = (head->data_len>>8) & 0xff;

//	strcopy(&cm_senddata[cm_nb_type_que],dat,len);  //去除最后的CRC校验和
	strcopy(&cm_senddata[cm_nb_type_que],dat,(len - 1));  //去除最后的CRC校验和
	cm_senddata[cm_nb_type_que+(len - 1)] = crc8_table(&cm_senddata[cm_nb_type_que],len-1);	 //从0xF6开始 
}

uint8_t crc;
void cm_decode(uint16_t len,uint8_t *rev)
{
	

    if(rev[cm_data_type_que] == 0x03)//03:下行
	{   
			crc = crc8_table(&rev[cm_nb_type_que],(len-cm_nb_type_que)-1);     //CRC校验从0xF6开始
			if(crc == rev[len-1])          //crc校验
			{
				if(strncmp((char*)&rev[cm_sn_que],(char*)&CM_DBLF_ID,8)==0)     //地表裂缝
				{
					cm_earth_decode(rev,len);
				}
                else if(strncmp((char*)&rev[cm_sn_que],(char*)&CM_QLF_ID,8)==0) //墙裂缝
				{
					cm_wall_decode(rev,len);
				}
				else if(strncmp((char*)&rev[cm_sn_que],(char*)&CM_DMQX_ID,8)==0)
				{
					cm_angle_decode(rev,len);
				}
				else if(strncmp((char*)&rev[cm_sn_que],(char*)&CM_RAIN_ID,8)==0)
				{
					cm_rain_decode(rev,len);
				}
				else if(strncmp((char*)&rev[cm_sn_que],(char*)&CM_MUD_ID,8)==0)                 
				{
					cm_mud_decode(rev,len);
				}
				else if(strncmp((char*)&rev[cm_sn_que],(char*)&CM_ANNU_ID,8)==0)  
				{
					cm_auto_decode(rev,len);
				}
                
			}
			else
			{
				rt_kprintf("CRC error\n");
			}
	    }
		else
		{
			rt_kprintf("ID error\n");
		}
}


/*地表裂缝解码函数*/
static void cm_earth_decode(uint8_t *rev,uint16_t len)
{
	float *p;
	uint32_t *i;
	uint16_t *d;
	uint8_t *j;
    uint8_t angle[4] = {0};
    uint8_t dist[4] = {0};
    rt_thread_t ctd;
    ctd = rt_thread_find("BC35-G");
	if(rev[cm_data_type_que] == 0x03)    //下行
	{
		if(rev[cm_cmd_que] == cm_angle_set)    //设置角度偏移告警阈值（地表/墙裂缝监测仪）
		{
			p = (float *)&rev[cm_info_que];     //添加对应设置值到全局变量或者读取全局变量值
            strcopy(angle,&rev[cm_info_que],4);    //目的是为解决浮点数4字节对齐问题
            p = (float*)angle;
            surface_angle  = *p;   //将平台值下传到模块。
            rev[cm_cmd_que] = cm_angle_up;  //改为上报角度偏移告警阈值（地表/墙裂缝监测仪）
            refreshkvalue();
		}
        
		else if(rev[cm_cmd_que] == cm_dist_set) //设置拉线偏移告警阈值（地表/墙裂缝监测仪）
		{
			p = (float *)&rev[cm_info_que];
            strcopy(dist,&rev[cm_info_que],4);   //目的是为解决浮点数4字节对齐问题
            p = (float*)dist;
            surface_dist  = *p;                 //将平台值下传到模块。
            rev[cm_cmd_que] = cm_dist_up;      //上报拉线偏移告警阈值（地表/墙裂缝监测仪）
            refreshkvalue();
		}
		else if(rev[cm_cmd_que] == cm_cycle_set)   //设置数据周期（地表/墙裂缝监测仪）
		{
			i = (uint32_t *)&rev[cm_info_que];
            cmgpscycle = *i;
			rt_kprintf("GPS采集周期=%d ms\n",*i);
            i = (uint32_t *)&rev[cm_info_que + 4];
            cmmotioncycle = *i;
            rt_kprintf("运动上报周期=%d ms\n",*i);
			i = (uint32_t *)&rev[cm_info_que + 8];
            cmcycle = *i;
			rt_kprintf("上传间隔=%d ms\n",*i);
            rev[cm_cmd_que] = cm_cycle_up;    //上报数据周期（地表/墙裂缝监测仪）
            refreshkvalue();
            rt_thread_resume(ctd);
		}
		else if(rev[cm_cmd_que] == cm_sample_set) //设置采样间隔 （地表/墙裂缝监测仪）
		{ 
			j = (uint8_t *)&rev[cm_info_que];
            if(*j == 0)
            {
                rt_kprintf("采样时间单位为秒");    
            }
			d = (uint16_t *)&rev[cm_info_que+1];
            cmsample = *d;
			rt_kprintf("采样间隔=%d\n",*d);
            rev[cm_cmd_que] = cm_sample_up;
            refreshkvalue();
		}
	}
//	cm_otadecodehandle(rev,len);   //OTA	
	rev[cm_data_type_que] = 0x00;   //改为上行
	rev[len-1]= crc8_table(&rev[cm_nb_type_que],(len-cm_nb_type_que)-1);
	if((rev[cm_cmd_que] != 0xF0) && (rev[cm_cmd_que] != 0xF1) && (rev[cm_cmd_que] != 0xF2) && (rev[cm_cmd_que] != 0xF3))
    {
        nb_send((char*)rev,len);  //上报数据(做OTA时不将接收的数据包上报给平台)    
    }
}

/*墙体裂缝解码函数*/
static void cm_wall_decode(uint8_t *rev,uint16_t len)
{
	float *p;
	uint32_t *i;
	uint16_t *d;
	uint8_t *j;
    uint8_t angle[4] = {0};
    uint8_t dist[4] = {0};
	if(rev[cm_data_type_que] == 0x03)    //下行
	{
		if(rev[cm_cmd_que] == cm_angle_set)    //设置角度偏移告警阈值（地表/墙裂缝监测仪）
		{
			p = (float *)&rev[cm_info_que];     //添加对应设置值到全局变量或者读取全局变量值
            strcopy(angle,&rev[cm_info_que],4);    //目的是为解决浮点数4字节对齐问题
            p = (float*)angle;
            wall_angle  = *p;   //将平台值下传到模块。
            rev[cm_cmd_que] = cm_angle_up;  //改为上报角度偏移告警阈值（地表/墙裂缝监测仪）
            refreshkvalue();
		}
        
		else if(rev[cm_cmd_que] == cm_dist_set) //设置拉线偏移告警阈值（地表/墙裂缝监测仪）
		{
			p = (float *)&rev[cm_info_que];
            strcopy(dist,&rev[cm_info_que],4);   //目的是为解决浮点数4字节对齐问题
            p = (float*)dist;
            wall_dist  = *p;                 //将平台值下传到模块。
            rev[cm_cmd_que] = cm_dist_up;      //上报拉线偏移告警阈值（地表/墙裂缝监测仪）
            refreshkvalue();
		}
		else if(rev[cm_cmd_que] == cm_cycle_set)   //设置数据周期（地表/墙裂缝监测仪）
		{
			i = (uint32_t *)&rev[cm_info_que];
            cmgpscycle = *i;
			rt_kprintf("GPS采集周期=%d ms\n",*i);
            i = (uint32_t *)&rev[cm_info_que + 4];
            cmmotioncycle = *i;
            rt_kprintf("运动上报周期=%d ms\n",*i);
			i = (uint32_t *)&rev[cm_info_que + 8];
            cmcycle = *i;
			rt_kprintf("上传间隔=%d ms\n",*i);
            rev[cm_cmd_que] = cm_cycle_up;    //上报数据周期（地表/墙裂缝监测仪）
            refreshkvalue();
		}
		else if(rev[cm_cmd_que] == cm_sample_set) //设置采样间隔 （地表/墙裂缝监测仪）
		{ 
			j = (uint8_t *)&rev[cm_info_que];
            if(*j == 0)
            {
                rt_kprintf("采样时间单位为秒");    
            }
			d = (uint16_t *)&rev[cm_info_que+1];
            cmsample = *d;
			rt_kprintf("采样间隔=%d\n",*d);
            rev[cm_cmd_que] = cm_sample_up;
            refreshkvalue();
		}
	}
//	cm_otadecodehandle(rev,len);   //OTA	
	rev[cm_data_type_que] = 0x00;   //改为上行
	rev[len-1]= crc8_table(&rev[cm_nb_type_que],(len-cm_nb_type_que)-1);
	if((rev[cm_cmd_que] != 0xF0) && (rev[cm_cmd_que] != 0xF1) && (rev[cm_cmd_que] != 0xF2) && (rev[cm_cmd_que] != 0xF3))
    {
        nb_send((char*)rev,len);  //上报数据(做OTA时不将接收的数据包上报给平台)    
    }
}
/*倾角解码函数*/
static void cm_angle_decode(uint8_t *rev,uint16_t len)
{
	float *p;
	uint32_t *i;
	uint16_t *d;
	uint8_t *j;
    uint8_t angle[4];
	if(rev[cm_data_type_que] == 0x03)    //下行
	{
//		rev[cm_msg_que]=cm_msg_ask;
		if(rev[cm_cmd_que] == cm_angle_set)    //设置角度偏移告警阈值（地面倾斜监测仪）
		{
			p = (float *)&rev[cm_info_que];//添加对应设置值到全局变量或者读取全局变量值
            strcopy(angle,&rev[cm_info_que],4);    //目的是为解决浮点数4字节对齐问题
            p = (float*)angle;
            dmqx_angle  = *p;   //将平台值下传到模块。
            
            rev[cm_cmd_que] = cm_angle_up;  //改为上报角度偏移告警阈值（地面倾斜监测仪）
            refreshkvalue();
		}
		else if(rev[cm_cmd_que] == cm_cycle_set)  //  设置数据周期（地面倾斜监测仪）
		{
			i = (uint32_t *)&rev[cm_info_que];
            cmgpscycle = *i;
			rt_kprintf("GPS采集周期=%d ms\n",*i);
			i = (uint32_t *)&rev[cm_info_que + 4];
            cmmotioncycle = *i;
            rt_kprintf("运动上报周期=%d ms\n",*i);
            i = (uint32_t *)&rev[cm_info_que + 8];
            cmcycle = *i;
			rt_kprintf("上传间隔=%d ms\n",*i);
            rev[cm_cmd_que] = cm_cycle_up;    //上报数据周期（地面倾斜监测仪）
            refreshkvalue();
		}
		else if(rev[cm_cmd_que] == cm_sample_set)   //设置采样间隔 （地面倾斜监测仪）
		{ 
			j = (uint8_t *)&rev[cm_info_que];
            if(*j == 0)
            {
                rt_kprintf("采样时间单位为秒");    
            }
			d = (uint16_t *)&rev[cm_info_que + 1];
            cmsample = *d;
			rt_kprintf("采样间隔=%d\n",*d);
            rev[cm_cmd_que] = cm_sample_up;    //上报采样间隔（地面倾斜监测仪）
            refreshkvalue();
		}
	}
//	cm_otadecodehandle(rev,len);   //OTA	
    rev[cm_data_type_que] = 0x00;   //改为上行
	rev[len-1]= crc8_table(&rev[cm_nb_type_que],(len-cm_nb_type_que)-1);
    if((rev[cm_cmd_que] != 0xF0) && (rev[cm_cmd_que] != 0xF1) && (rev[cm_cmd_que] != 0xF2) && (rev[cm_cmd_que] != 0xF3))
    {
        nb_send((char*)rev,len);  //上报数据(做OTA时不将接收的数据包上报给平台)    
    }
	
}

/*雨量解码函数*/
static void cm_rain_decode(uint8_t *rev,uint16_t len)
{
	float *p;
	uint32_t *i;
	uint16_t *d;
	uint8_t *j;
    
    uint8_t rain[4];
	if(rev[cm_data_type_que] == 0x03)    //下行
	{
//		rev[cm_msg_que]=cm_msg_ask;
		if(rev[cm_cmd_que] == cm_rain_threshold_set)    //设置告警阈值 (雨量) 数据扩大10倍传输  2个字节
		{
            cmrain = rev[cm_info_que] | (rev[cm_info_que + 1] << 8);  //低位在前，高位在后
//			p = (float *)&rev[cm_info_que];//添加对应设置值到全局变量或者读取全局变量值
//            strcopy(rain,&rev[cm_info_que],4);    //目的是为解决浮点数4字节对齐问题
//            p = (float*)rain;
//            cmrain = *p;
            cmrain = cmrain/10 ; //缩小10倍(因为上报时要放大10倍，第一次上报阈值也需放大10倍)
            rev[cm_cmd_que] = cm_rain_threshold_up;  //改为上报告警阈值（雨量）数据扩大10倍传输
            refreshkvalue();
           
            
		}
		else if(rev[cm_cmd_que] == cm_rain_cycle_set)
		{
			i = (uint32_t *)&rev[cm_info_que];
            cmgpscycle = *i;
			rt_kprintf("GPS采集周期=%d ms\n",*i);
            i = (uint32_t *)&rev[cm_info_que + 4];
            cmmotioncycle = *i;
            rt_kprintf("运动上传间隔=%d ms\n",*i);
			i = (uint32_t *)&rev[cm_info_que + 8];
            cmcycle = *i;
			rt_kprintf("上传间隔=%d ms\n",*i);
            rev[cm_cmd_que] = cm_rain_cycle_up;    //上报数据周期（地表/墙裂缝监测仪）
            refreshkvalue();
		}
		else if(rev[cm_cmd_que] == cm_rain_sample_set)   //设置采样间隔（雨量）
		{ 
			j = (uint8_t *)&rev[cm_info_que];
            if(*j == 0)
            {
                rt_kprintf("采样时间单位为秒");    
            }
			d = (uint16_t *)&rev[cm_info_que + 1];
            cmsample = *d;
			rt_kprintf("采样时间=%d\n",*d);
            rev[cm_cmd_que] = cm_rain_sample_up;    //上报采样间隔（雨量）
            refreshkvalue();
		}
	}
//    cm_otadecodehandle(rev,len);   //OTA	
	rev[cm_data_type_que] = 0x00;   //改为上行
	rev[len-1]= crc8_table(&rev[cm_nb_type_que],(len-cm_nb_type_que)-1);
    
	if((rev[cm_cmd_que] != 0xF0) && (rev[cm_cmd_que] != 0xF1) && (rev[cm_cmd_que] != 0xF2) && (rev[cm_cmd_que] != 0xF3))
    {
        nb_send((char*)rev,len);  //上报数据(做OTA时不将接收的数据包上报给平台)    
    }    
}

/*泥位解码函数*/
static void cm_mud_decode(uint8_t *rev,uint16_t len)
{
	float *p;
	uint32_t *i;
	uint16_t *d;
	uint8_t *j;
    uint8_t mud[4];
	if(rev[cm_data_type_que] == 0x03)    //下行
	{
//		rev[cm_msg_que]=cm_msg_ask;
		if(rev[cm_cmd_que] == cm_mud_threshold_set)    //设置泥位告警阈值1个字节
		{
			p = (float *)&rev[cm_info_que];//添加对应设置值到全局变量或者读取全局变量值
            strcopy(mud,&rev[cm_info_que],2);    //目的是为解决浮点数4字节对齐问题
            p = (float*)mud;
            mudthreshold  = *p;   //将平台值下传到模块。
            rev[cm_cmd_que] = cm_mud_threshold_up;  //改为上报泥位告警阈值
             refreshkvalue();
		}
		else if(rev[cm_cmd_que] == cm_cycle_set) //设置数据周期（泥位）
		{
			i = (uint32_t *)&rev[cm_info_que];
            cmgpscycle = *i;
			rt_kprintf("GPS采集周期=%d ms\n",*i);
            i = (uint32_t *)&rev[cm_info_que + 4];
            cmmotioncycle = *i;
            rt_kprintf("运动上报周期=%d ms\n",*i);
			i = (uint32_t *)&rev[cm_info_que + 8];
            cmcycle = *i;
			rt_kprintf("上传间隔=%d ms\n",*i);
            rev[cm_cmd_que] = cm_cycle_up;    //上报数据周期（泥位）
            refreshkvalue();
		}
		else if(rev[cm_cmd_que] == cm_sample_set) //设置采样间隔 （泥位） 
		{ 
			j = (uint8_t *)&rev[cm_info_que];
             if(*j == 0)
            {
                rt_kprintf("采样时间单位为秒");    
            }
//			rt_kprintf("采样时间单位=%x\n",*j);
			d = (uint16_t *)&rev[cm_info_que + 1];
            cmsample = *d;
			rt_kprintf("采样间隔=%d\n",*d);
            rev[cm_cmd_que] = cm_sample_up;    //上报采样间隔（泥位）
            refreshkvalue();
		}
	}
//	cm_otadecodehandle(rev,len);   //OTA	
	rev[cm_data_type_que] = 0x00;   //改为上行
	rev[len-1]= crc8_table(&rev[cm_nb_type_que],(len-cm_nb_type_que)-1);
	if((rev[cm_cmd_que] != 0xF0) && (rev[cm_cmd_que] != 0xF1) && (rev[cm_cmd_que] != 0xF2) && (rev[cm_cmd_que] != 0xF3))
    {
        nb_send((char*)rev,len);  //上报数据(做OTA时不将接收的数据包上报给平台)    
    } 
}

/*智能设备解码函数*/
extern void read_data(uint8_t *data,uint8_t size);
uint8_t vlum[8] = {0x7E,0xFF,0x06,0x05,0x00,0x00,0x00,0xEF};    //音量减
uint8_t music[8] = {0x7E,0xFF,0x06,0x08,0x00,0x00,0x01,0xEF};
uint8_t close_music[8] = {0x7E,0xFF,0x06,0x16,0x00,0x00,0x00,0xEF};  //停止播放
uint8_t close_shark[8] = {0x7E,0xFF,0x06,0x3A,0x00,0x00,0x01,0xEF};   //闪灯关
static void cm_auto_decode(uint8_t *rev,uint16_t len)
{
	uint32_t *i;
	uint16_t *d;
	uint8_t *power_switch;
    uint8_t *voice;
    uint8_t *j;
	if(rev[cm_data_type_que] == 0x03)    //下行
	{
//		rev[cm_msg_que]=cm_msg_ask;
		if(rev[cm_cmd_que] == cm_annu_set)    //设置智能报警器参数（智能报警器）
		{
			j = (uint8_t *)&rev[cm_info_que];//添加对应设置值到全局变量或者读取全局变量值
            cmautos = *j;
			j = (uint8_t *)&rev[cm_info_que + 1];
            cmautolang = *j;
            if(cmautos == 0 && cmautolang == 0)     //打开电源,关闭音乐
			{
				read_data(close_music,8); //关闭音乐
				rt_thread_delay(100);     
				read_data(close_shark,8); //关闭闪光灯
			}
			else if(cmautos == 0 && cmautolang != 0)
			{
				music[6] = cmautolang;
				read_data(music,8);  //播放音乐
			}
            rev[cm_cmd_que] = cm_annu_up;  //上报智能报警器参数（智能报警器）
            refreshkvalue();
		}
		else if(rev[cm_cmd_que] == cm_annu_cycle_set) //设置数据周期 (智能报警器)
		{
			i = (uint32_t *)&rev[cm_info_que];
            cmgpscycle = *i;
			rt_kprintf("GPS采集周期=%d ms\n",*i);
            i = (uint32_t *)&rev[cm_info_que + 4];
            cmmotioncycle = *i;
            rt_kprintf("运动上报周期=%d ms\n",*i);
			i = (uint32_t *)&rev[cm_info_que + 8];
            cmcycle = *i;
			rt_kprintf("上传间隔=%d ms\n",*i);
            rev[cm_cmd_que] = cm_annu_cycle_up;    //上报数据周期 (智能报警器)
            refreshkvalue();
		}
		else if(rev[cm_cmd_que] == cm_annu_sample_set)   //  设置采样间隔 (智能报警器)
		{ 
			j = (uint8_t *)&rev[cm_info_que];
            if(*j == 0)
            {
                rt_kprintf("采样时间单位为秒");    
            }
//			rt_kprintf("采样时间单位=%x\n",*j);
			d = (uint16_t *)&rev[cm_info_que + 1];
            cmsample = *d;
			rt_kprintf("采样间隔=%d\n",*d);
            rev[cm_cmd_que] = cm_annu_sample_up;    //上报采样间隔（智能报警器）
            refreshkvalue();
		}
	}
//    cm_otadecodehandle(rev,len);   //OTA
	rev[cm_data_type_que] = 0x00;   //改为上行
	rev[len-1]= crc8_table(&rev[cm_nb_type_que],(len-cm_nb_type_que)-1);
    if((rev[cm_cmd_que] != 0xF0) && (rev[cm_cmd_que] != 0xF1) && (rev[cm_cmd_que] != 0xF2) && (rev[cm_cmd_que] != 0xF3))
    {
        nb_send((char*)rev,len);  //上报数据(做OTA时不将接收的数据包上报给平台)    
    } 
}

void cm_device_init(cm_header_t ptr,uint16_t length)
{
    #if USING_DBLH
    cm_default_config.dev_type = CM_EARTH_DEV;     //设备类型改为地表裂缝监测仪
    cm_default_config.dev_id = CM_DBLF_ID;
    config_data.cmd = cm_softversion_up;
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报软件版本
    rt_thread_delay(1000*5);
    config_data.cmd = cm_modulestatus_up;  
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报模块状态(地表裂缝监测仪)
    rt_thread_delay(1000*5);
    config_data.cmd = cm_cycle_up;      
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报数据周期
    rt_thread_delay(1000*5);
    config_data.cmd = cm_angle_up;      
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报角度偏移告警阈值(地表裂缝监测仪)
    rt_thread_delay(1000*5);
    config_data.cmd = cm_dist_up;      
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报拉线偏移告警阈值(地表裂缝监测仪)
    rt_thread_delay(1000*5);             
    config_data.cmd = cm_sample_up;     
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报采样间隔
    rt_thread_delay(1000*5);
    #endif

    #if USING_QLH
    cm_default_config.dev_type = CM_WALL_DEV;     //设备类型改为墙裂缝监测仪
    cm_default_config.dev_id = CM_QLF_ID;
    config_data.cmd = cm_softversion_up;
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报软件版本
    rt_thread_delay(1000*5);
    config_data.cmd = cm_modulestatus_up;  
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报模块状态(墙裂缝监测仪)
    rt_thread_delay(1000*5);
    config_data.cmd = cm_cycle_up;      
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报数据周期
    rt_thread_delay(1000*5);
    config_data.cmd = cm_angle_up;      
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报角度偏移告警阈值（墙裂缝监测仪）
    rt_thread_delay(1000*5);
    config_data.cmd = cm_dist_up;      
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报拉线偏移告警阈值(地表裂缝监测仪)
    rt_thread_delay(1000*5);
    config_data.cmd = cm_sample_up;
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报采样间隔
    rt_thread_delay(1000*5);
    #endif

    #if USING_DMQX

    cm_default_config.dev_type = CM_ANGLE_DEV;     //设备类型改为地面倾斜监测仪
    cm_default_config.dev_id = CM_DMQX_ID;         //设备ID改为地面倾斜监测仪
    config_data.cmd = cm_softversion_up;
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报软件版本
    rt_thread_delay(1000*5);
    config_data.cmd = cm_modulestatus_up;  
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报模块状态(地面倾斜监测仪)
    rt_thread_delay(1000*5);
    config_data.cmd = cm_cycle_up;      
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);   // 上报数据周期
    rt_thread_delay(1000*5);
    config_data.cmd = cm_angle_up;      
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报角度阈值(地面倾斜监测仪)
    rt_thread_delay(1000*5);
    config_data.cmd = cm_sample_up;
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报采样间隔
    rt_thread_delay(1000*5);
    #endif

    #if USING_MUD
    cm_default_config.dev_type = CM_MUD_DEV;     //设备类型改为泥位计
    cm_default_config.dev_id = CM_MUD_ID;         //设备ID改为泥位计
    config_data.cmd = cm_softversion_up;
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报软件版本
    rt_thread_delay(1000*5);
    config_data.cmd = cm_modulestatus_up;  
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报模块状态(泥位计)
    rt_thread_delay(1000*5);
    config_data.cmd = cm_cycle_up;      
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报数据周期
    rt_thread_delay(1000*5);
    config_data.cmd = cm_mud_threshold_up;      
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报泥位告警阈值
    rt_thread_delay(1000*5);
    config_data.cmd = cm_sample_up;
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报采样间隔
    rt_thread_delay(1000*5);
    #endif

    #if USING_RAIN
    cm_default_config.dev_type = CM_RAIN_DEV;     //设备类型雨量计
    cm_default_config.dev_id = CM_RAIN_ID;
    config_data.cmd = cm_rain_softversion_up;
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报软件版本（雨量计）
    rt_thread_delay(1000*5);
    config_data.cmd = cm_modulestatus_up;  
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报模块状态(雨量计)
    rt_thread_delay(1000*5);
    config_data.cmd = cm_rain_cycle_up;      
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);      // 上报数据周期(雨量计)
    rt_thread_delay(1000*5);
    config_data.cmd = cm_rain_threshold_up;     
    length = cm_encode(ptr,&config_data);    
    nb_send((char*)cm_senddata,length);   //上报告警阈值 (雨量)
    rt_thread_delay(1000*5);
    config_data.cmd = cm_rain_sample_up;
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报采样间隔(雨量)
    rt_thread_delay(1000*5);
    #endif

    #if USING_ALARMER
    cm_default_config.dev_type = CM_AUTO_DEV;     //设备类型改为智能报警器
    cm_default_config.dev_id = CM_ANNU_ID; 
    config_data.cmd = cm_annu_softversion_up;
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报软件版本
    rt_thread_delay(1000*5);
    config_data.cmd = cm_modulestatus_up;  
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报模块状态(智能报警器)
    rt_thread_delay(1000*5);
    config_data.cmd = cm_annu_cycle_up;     
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报数据周期
    rt_thread_delay(1000*5);
    config_data.cmd = cm_annu_up;      
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报智能报警器参数
    rt_thread_delay(1000*5);
    config_data.cmd = cm_annu_sample_up;
    length = cm_encode(ptr,&config_data);
    nb_send((char*)cm_senddata,length);  //上报智能报警器采样间隔
    rt_thread_delay(1000*5);
    #endif
}

//static void cm_otadecodehandle(uint8_t *rev,uint16_t len)
//{   uint32_t delayms = 0;
//	uint32_t *i;
//	uint16_t *d;
//	uint16_t n;
//	uint32_t indx = 0;
//    uint8_t retry = 4;
//    if(rev[cm_cmd_que] == cm_server_request_ota)      //服务器向设备发送升级请求下行
//    {
//        d = (uint16_t *)&rev[cm_info_que];    //info数据起始字节   
//        cm_soft_ver = *d;                   //最新软件版本2byte
//        rt_kprintf("服务器推送升级版本 = %4x\n",cm_soft_ver);
//        i = (uint32_t *)&rev[cm_info_que + 2];       //升级包总长度 4byte
//        rt_kprintf("升级包总长度 = %d\n",*i);
//        cm_packet_len = *i;                          //升级包总长度 4byte
//        i = (uint32_t *)&rev[cm_info_que + 6];      //CRC 4byte
//        rt_kprintf("升级文件CRC32校验 = %8x\n",*i);
//        cm_crc32 = *i;                               //升级文件CRC校验    
//        i = (uint32_t *)&rev[cm_info_que + 10];    //分包总个数
//        rt_kprintf("分包总数 = %d\n",*i);
//        cm_packet_num = *i;                        //分包总个数
//        cm_rand_crc = 0;                          //分包码累加和
//        config_data.pack_num = 1;                   //设备端每次请求一个包
//        cm_pack_index = 0;
//        config_data.pack_index = cm_pack_index;             //请求发送包的索引0(起始值)
//        cm_default_config.msg_mode = cm_msg_up;     //上行    
//        config_data.cmd = cm_client_request_otadata;            //设备向服务器请求升级数据    cm_client_request_otadata    上行
//        cm_default_config.user_data = &config_data;             //config_data 第二层数据,cm_default_config :header
//        n = cm_encode(&cm_default_config,&config_data);
//        nb_send_ota((char*)cm_senddata,n);                         //终端请求服务器发送第一个包（发送）
//        for(delayms = 0;delayms < 8000000;delayms++);     //100ms
//        rt_kprintf("终端请求服务器发送第一个包\n");
//        cm_default_config.msg_mode = cm_msg_up; //改为上行
//    }
//        
//    if(rev[cm_cmd_que] == cm_server_send_otadata)               //服务器向设备发送升级数据下行
//    {
////        rt_timer_start(&otatimer); //启动定时器      重传机制25S
//        do{
//            i = (uint32_t *)&rev[cm_info_que];                       //当前下发升级包索引 4byte
//            indx = *i;
//            rt_kprintf("当前下发升级包索引 = %d\n",indx);
//            i = (uint32_t *)&rev[cm_info_que + 4];                  //当前下发升级包长度 4byte
//            rt_kprintf("当前下发升级包长度 = %d\n",*i);
//            i = (uint32_t *)&rev[cm_info_que + 8];                //当前升级包CRC32校验  4byte
//            rt_kprintf("当前升级包CRC32校验 = %8x\n",*i);
//            d = (uint16_t*)&rev[cm_info_que + 12];               // 分包码，需将接收到的分包码求和上传，2byte 
//            rt_kprintf("服务器随机码 = %4x\n",*d);
//            if(cm_pack_index == indx)
//            {
//                rt_timer_stop(&otatimer); //启动定时器
//                break;                      //跳出循环 
//            }    
//                                                      //跳出循环
//           for(delayms = 0;delayms < 80000000;delayms++);     //1s
//           for(delayms = 0;delayms < 80000000;delayms++);     //1s
//           for(delayms = 0;delayms < 80000000;delayms++);     //1s
//        }while(retry--);                                       //移动的重传机制，重传三次,还需添加超时机制 上行时推出
//        
//        if(cm_pack_index == indx)   //请求升级包索引和接收到的一致,且分包数不小于1
//        {           
//            cm_packet_num--;
//            cm_pack_index++;
//            config_data.pack_num = 1;
//            config_data.pack_index = cm_pack_index;
//            rt_kprintf("剩余总包数 = %d,下一请求索引号 = %d\n",cm_packet_num,config_data.pack_index);
//            cm_rand_crc = (uint32_t)(cm_rand_crc + (*d));
//            rt_kprintf("服务器随机码累加 = %4x\n",cm_rand_crc);
//            if(cm_packet_num != 0)
//            {
//                for(delayms = 0;delayms < 80000000;delayms++); 
//                cm_default_config.msg_mode = cm_msg_up;      //改为上行
//                config_data.cmd = cm_client_request_otadata; //设备向服务器请求升级数据
//                config_data.pack_num = 1;                   //设备端每次请求一个包
//                config_data.pack_index = cm_pack_index;
//                cm_default_config.user_data = &config_data;
//                n = cm_encode(&cm_default_config,&config_data);
// 
//                nb_send_ota((char*)cm_senddata,n);     //设备向服务器请求索引为cm_pack_index的1个包的数据，发送下个数据包
//                rt_timer_start(&otatimer);
//            }
//        }
//        else
//        {
//            cm_default_config.msg_mode = cm_msg_up;      //改为上行
//            config_data.cmd = cm_client_request_otadata; //设备向服务器请求升级数据
//            config_data.pack_num = 1;                   //设备端每次请求一个包
//            config_data.pack_index = cm_pack_index;
//            cm_default_config.user_data = &config_data;
//            n = cm_encode(&cm_default_config,&config_data);
//            nb_send_ota((char*)cm_senddata,n);     //设备向服务器请求索引为cm_pack_index的1个包的数据。
//            rt_timer_start(&otatimer);
//            
////            cm_default_config.msg_mode = cm_msg_up; //上行
////            config_data.cmd = cm_ota_status_up;          //设备上报远程升级状态
////            config_data.ota_sum = cm_rand_crc;      //分包码累加和
////            config_data.ota_lab = 1;                //1数据传输失败 ，放弃本次升级   
////            cm_default_config.user_data = &config_data;
////            n = cm_encode(&cm_default_config,&config_data);
////            nb_send_ota((char*)cm_senddata,n);
//        }
//        if(cm_packet_num == 0)
//        {
//            rt_kprintf("升级包接收完成 %d\n",cm_pack_index);
//            cm_default_config.msg_mode = cm_msg_up; //上行
//            config_data.cmd = cm_ota_status_up;          //设备上报远程升级状态
//            config_data.ota_sum = cm_rand_crc;      //分包码累加和
//            config_data.ota_lab = 2;                //2数据传输成功    
//            cm_default_config.user_data = &config_data;
//            n = cm_encode(&cm_default_config,&config_data);
//            nb_send_ota((char*)cm_senddata,n);
//            
//            cm_default_config.msg_mode = cm_msg_up; //上行
//            #if (USING_DBLH | USING_DMQX | USING_QLH | USING_MUD)
//                config_data.cmd = cm_softversion_up;     
//            #endif
//            #if USING_ALARMER
//                config_data.cmd = cm_annu_softversion_up;    
//            #endif
//            #if USING_RAIN
//                config_data.cmd = cm_rain_softversion_up;      
//            #endif
////                    config_data.cmd = cm_rain_softversion_up;       //cm_annu_softversion_up = 0x10,上报软件版本 (智能报警器)
//            cm_default_config.user_data = &config_data;
//            n = cm_encode(&cm_default_config,&config_data);
//            nb_send((char*)cm_senddata,n);
//            refreshkvalue();
//        }

//    }
//}   

///*common ota up hander*/
//static void cm_otaencode_handle(cm_header_t head,cm_mainmsg_t msg,uint8_t *userdata)
//{
//    if(msg->cmd == cm_client_request_otadata)       //设备向服务器请求升级数据
//    {
//        head->data_len = 12;
//        userdata[0] = 0xF6;
//        userdata[1] = 0x09;  //
//        userdata[2] = cm_client_request_otadata;  //  0xF2
//        
//        userdata[3] = msg->pack_index & 0x000000FF;  //分包索引                
//        userdata[4] = msg->pack_index >>8 & 0x000000FF;
//        userdata[5] = msg->pack_index >>16 & 0x000000FF;
//        userdata[6] = msg->pack_index >>24 & 0x000000FF; //index会变（msg->pack_index ++）
//        
//        userdata[7] = msg->pack_num & 0x000000FF;             
//        userdata[8] = msg->pack_num>>8 & 0x000000FF;
//        userdata[9] = msg->pack_num>>16 & 0x000000FF;
//        userdata[10] = msg->pack_num>>24 & 0x000000FF;    // 当前请求分包个数，分包个数一直为1,每次请求一个包。
//   
//    }
//    else if(msg->cmd == cm_ota_status_up) // 设备上报远程升级状态
//    {
//        head->data_len = 9;
//        userdata[0] = 0xF6;
//        userdata[1] = 0x06;  //
//        userdata[2] = cm_ota_status_up;  //  0xF0
//        userdata[3] = msg->ota_lab;     // ota_lab :1、设备放弃本次升级，升级失败；2：本次升级数据包传输成功 
//        userdata[4] = msg->ota_sum & 0x000000FF;      //ota_sum
//        userdata[5] = msg->ota_sum >>8 & 0x000000FF;             
//        userdata[6] = msg->ota_sum >>16 & 0x000000FF;
//        userdata[7] = msg->ota_sum >>24 & 0x000000FF; //cm_rand_crc:分包码累加和    
//    }    
//}    

//extern struct rt_event nb_event; 
//void otatimer_callback(void* parameter)
//{
//    rt_err_t err;
//	rt_kprintf("\\*****************OTA 定时器超时*****************\\\r\n");
//    otaflag = 1;
//    otaflag ++;
//    err = rt_event_send(&nb_event,0x100);
//    if(err !=RT_EOK)
//    {
//        rt_kprintf("ERROR!!!!!!!!\r\n");
//    }
//}

//void ota_thread_entry(void *args)
//{    
////    uint32_t reque_count = 3;
//    uint16_t n = 0;
//    while(1)
//    {
//        rt_event_recv(&nb_event,1<<8,RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER, RT_NULL);
//        rt_kprintf("\\********1243*********OTA 定时器超时*****************\\\r\n");
//        if(otaflag <= 3)
//        {
//            cm_default_config.msg_mode = cm_msg_up;      //改为上行
//            config_data.cmd = cm_client_request_otadata; //设备向服务器请求升级数据
//            config_data.pack_num = 1;                   //设备端每次请求一个包
//            config_data.pack_index = cm_pack_index;
//            cm_default_config.user_data = &config_data;
//            n = cm_encode(&cm_default_config,&config_data);
//            nb_send_ota((char*)cm_senddata,n);     //设备向服务器请求索引为cm_pack_index的1个包的数据。
//            rt_timer_start(&otatimer);
//        }
//        if(otaflag > 3)
//        {
//            otaflag = 0;        //清零
//            uint8_t n;
//            cm_default_config.msg_mode = cm_msg_up; //上行
//            config_data.cmd = cm_ota_status_up;          //设备上报远程升级状态
//            config_data.ota_sum = cm_rand_crc;      //分包码累加和
//            config_data.ota_lab = 1;                //1数据传输失败    
//            cm_default_config.user_data = &config_data;
//            n = cm_encode(&cm_default_config,&config_data);
//            nb_send_ota((char*)cm_senddata,n);
////            for(uint32_t delayms = 0;delayms < 80000000;delayms++);
//            rt_thread_delay(1000 * 1);
//            rt_kprintf("定时器到25S,放弃本次升级\r\n");
//            rt_timer_stop(&otatimer);
//        }
//    }
//}