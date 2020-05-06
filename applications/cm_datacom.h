#ifndef __CM_DATACOM_H__
#define __CM_DATACOM_H__
#include "stm32l476xx.h"
#include <rtthread.h>
#include "devtype.h"

/* select which device is used */
#define USING_DBLH                     0       //确认是否使用地表裂缝仪
#define USING_QLH                      0      //确认是否使用墙裂缝仪
#define USING_DMQX                     1       //确认是否使用地面倾斜监测仪
#define USING_MUD                      0      //确认是否使用地面泥位计
#define USING_RAIN                     0      //确认是否使用雨量计
#define USING_ALARMER                  0      //确认是否使用智能报警器 

/* device id */
#define    CM_DBLF_ID   "11111111"      //地表裂缝设备ID
#define    CM_QLF_ID   "22222222"      //墙裂缝设备ID
#define    CM_DMQX_ID   "333333333"      //地面倾斜设备ID
#define    CM_MUD_ID   "44444444"      //泥位计设备ID
#define    CM_RAIN_ID   "55555555"      //雨量计设备ID
#define    CM_ANNU_ID   "66666666"      //智能报警器设备ID

/* device type */
#define CM_EARTH_DEV            0xA1       //地表裂缝
#define CM_WALL_DEV             0xA2       //墙裂缝
#define CM_ANGLE_DEV            0xA3       //倾角
#define CM_RAIN_DEV             0xA4       //雨量
#define CM_MUD_DEV              0xA5       // 泥位
#define CM_AUTO_DEV             0xA6        //智能

/* data struct index */
#define  cm_head_que           0
#define  cm_count_que          1
#define  cm_spare_byte         2
#define  cm_data_type_que      3
#define  cm_sn_que             4
#define  cm_datalen1_que      12    //2个字节
#define  cm_nb_type_que       14
#define  cm_datalen2_que      15   //1个字节
#define  cm_cmd_que           16
#define  cm_info_que          17

struct cm_configkv
{
//	float angle;
//	float wire;
//	uint32_t cycle;
//	uint16_t softversion;  //
//	uint16_t sample;
//	uint16_t rain;
//    float mud;
//	uint8_t autos;
//	uint8_t autolang;
    
//    float angle;      //打算细分
//	float wire;
	uint32_t cycle;
	uint32_t softversion; //16
	uint32_t sample;   //16
	float rain;        //16
	uint32_t autos;    //8
	uint32_t autolang;  //8
    float mud;         //16
    float dmqx_angle;   //地面倾斜
    float surface_ang;  //地表角度阈值
    float surface_dis;  //地表拉线距离
    float wall_ang;     //墙角度阈值
    float wall_dis;     //墙拉线阈值
};

/* struct main message value */
struct cm_mainmsg{
 
    uint8_t cmd; //第二层数据中的CMD,我自行添加，用于区分上报类型和设置/读取类型
	char *date;            //日期 7Byte
	char *time;            //时间 7Byte
	float dir_x;           //X轴方向位移量 4Byte
	float dir_y;           //Y轴方向位移量 4Byte
	float dir_z;           //Z轴方向位移量 4Byte
	float angle_x;         //X轴倾角 4Byte
	float angle_y;          //Y轴倾角 4Byte
	float angle_z;         //Z轴倾角 4Byte
	float dist;            //拉线长度 4Byte
	uint8_t bat_per;        //电量百分比 1Byte
	float bat_vol;          //电压 4Byte
	float gps_long;    //GPS经度   4Byte
	float gps_lat;     //GPS纬度   4Byte
	float bd_long;     //BD经度   4Byte
	float bd_lat;     //BD纬度   4Byte
	uint16_t signal;    //信号 2Byte
	float rain_n;       //雨量强度   4Byte
    float mud_n;         //泥位 4Byte 
	uint8_t auto_s;       //智能报警器电源开关 1byte
	uint8_t auto_lang;    //智能报警器播放语音 1byte
    uint32_t pack_index;       //ota 4byte 分包索引
	uint32_t pack_num;         //ota 4byte 分包个数设定一直为1。
	uint8_t ota_lab;      //ota 标记
	uint32_t ota_sum;     //ota  分包码累加和
};
typedef struct cm_mainmsg *cm_mainmsg_t;
/* data stuct */
struct cm_header{

	uint8_t version;
    uint8_t dev_type;  //将2字节的保留count改为设备类型
    uint8_t spare;      //保留一个字节
//    uint16_t count;
    uint8_t  msg_mode;
    char     *dev_id;     //8字节
    uint16_t  data_len; //2个字节
    void     *user_data; //第二层数据	
};
typedef struct cm_header *cm_header_t;
/*定义消息传递类型（上行或者下行）*/
enum cm_msg_id
{
	cm_msg_up = 0x00,
	cm_msg_upack = 0x01,
	cm_msg_down = 0x03,

};

/* list cmd values*/
enum cm_cmd_word
{
	cm_modulestatus_up = 0x01,       //上报模块状态（地表/墙裂缝监测仪、地面倾斜检测仪、泥位、雨量、智能报警器）
    cm_angle_read = 0x0F,                    //读取角度偏移告警阈值（地表/墙裂缝监测仪、地面倾斜检测仪）
    cm_angle_up = 0x10,                      //上报角度偏移告警阈值（地表/墙裂缝监测仪、地面倾斜检测仪）
    cm_angle_set = 0x11,                      //设置角度偏移告警阈值（地表/墙裂缝监测仪、地面倾斜检测仪）
    cm_dist_read = 0x15,                      //读取拉线偏移告警阈值（地表/墙裂缝监测仪）
    cm_dist_up = 0x16,                      //上报拉线偏移告警阈值（地表/墙裂缝监测仪）
    cm_dist_set = 0x17,                      //设置拉线偏移告警阈值（地表/墙裂缝监测仪）
    cm_cycle_read = 0x40,                     //读取数据周期（地表/墙裂缝监测仪、地面倾斜检测仪、泥位）
    cm_cycle_up = 0x41,                     //上报数据周期（地表/墙裂缝监测仪、地面倾斜检测仪、泥位）
    cm_cycle_set = 0x42,                     //设置数据周期（地表/墙裂缝监测仪、地面倾斜检测仪、泥位）
    cm_softversion_read = 0x43,              //  读取软件版本 （地表/墙裂缝监测仪、地面倾斜检测仪、泥位）
    cm_softversion_up = 0x44,              //  上报软件版本 （地表/墙裂缝监测仪、地面倾斜检测仪、泥位）
    cm_sample_read = 0x46,                 //  读取采样间隔 （地表/墙裂缝监测仪、地面倾斜检测仪、泥位）
    cm_sample_up = 0x47,                 //  上报采样间隔 （地表/墙裂缝监测仪、地面倾斜检测仪、泥位）
    cm_sample_set = 0x48,                //  设置采样间隔 （地表/墙裂缝监测仪、地面倾斜检测仪、泥位）
    cm_heartbeat_up = 0x37,               //  上报心跳包 （地表/墙裂缝监测仪、地面倾斜检测仪、泥位、智能报警器）
    
    cm_ota_status_up = 0xF0,             // 设备上报远程升级状态（地表/墙裂缝监测仪、地面倾斜检测仪、泥位、雨量、智能报警器）
    cm_server_request_ota = 0xF1,        //  服务器向设备发送升级请求（地表/墙裂缝监测仪、地面倾斜检测仪、泥位、雨量、智能报警器）  
    cm_client_request_otadata = 0xF2,   //    设备向服务器请求升级数据  （地表/墙裂缝监测仪、地面倾斜检测仪、泥位、雨量、智能报警器）
    cm_server_send_otadata = 0xF3,      //    服务器向设备发送升级数据    （地表/墙裂缝监测仪、地面倾斜检测仪、泥位、雨量、智能报警器）
    
    cm_rain_cycle_read = 0x0C,         //读取数据周期 (雨量)
    cm_rain_cycle_up = 0x0D,           // 上报数据周期 (雨量)
    cm_rain_cycle_set = 0x0E,           // 设置数据周期 (雨量)
    cm_rain_softversion_read = 0x0F,    //  读取软件版本 (雨量)
    cm_rain_softversion_up = 0x10,    //  上报软件版本 (雨量)
    cm_rain_sample_read = 0x12,    //  读取采样间隔 (雨量)
    cm_rain_sample_up = 0x13,    //  上报采样间隔 (雨量)
    cm_rain_sample_set = 0x14,    //  设置采样间隔 (雨量)
    cm_rain_threshold_read = 0x19,       //读取告警阈值(雨量)
    cm_rain_threshold_up = 0x1A,       //上报告警阈值 (雨量)
    cm_rain_threshold_set = 0x1B,       //设置告警阈值 (雨量)
    
    cm_annu_read = 0x09,               // 读取智能报警器参数（智能报警器）
    cm_annu_up = 0x0A,                //上报智能报警器参数（智能报警器）
    cm_annu_set = 0x0B,                //设置智能报警器参数（智能报警器）
    cm_annu_cycle_read = 0x0C,          //读取数据周期 (智能报警器)
    cm_annu_cycle_up = 0x0D,          //上报数据周期 (智能报警器)
    cm_annu_cycle_set = 0x0E,          //设置数据周期 (智能报警器)
    cm_annu_softversion_read = 0x0F,      //  读取软件版本 (智能报警器)
    cm_annu_softversion_up = 0x10,      //  上报软件版本 (智能报警器)
    cm_annu_sample_read = 0x12,    //  读取采样间隔 (智能报警器)
    cm_annu_sample_up = 0x13,    //  上报采样间隔 (智能报警器)
    cm_annu_sample_set = 0x14,    //  设置采样间隔 (智能报警器)
   
    cm_mud_threshold_read = 0x0F,       // 读取泥位告警阈值
    cm_mud_threshold_up = 0x10,        // 上报泥位告警阈值
    cm_mud_threshold_set = 0x11,       // 设置泥位告警阈值

};

uint16_t cm_encode(cm_header_t head,cm_mainmsg_t msg);
void cm_decode(uint16_t len,uint8_t *rev);
void cm_device_init(cm_header_t ptr,uint16_t length);

#endif



















