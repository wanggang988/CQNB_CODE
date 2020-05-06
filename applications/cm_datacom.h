#ifndef __CM_DATACOM_H__
#define __CM_DATACOM_H__
#include "stm32l476xx.h"
#include <rtthread.h>
#include "devtype.h"

/* select which device is used */
#define USING_DBLH                     0       //ȷ���Ƿ�ʹ�õر��ѷ���
#define USING_QLH                      0      //ȷ���Ƿ�ʹ��ǽ�ѷ���
#define USING_DMQX                     1       //ȷ���Ƿ�ʹ�õ�����б�����
#define USING_MUD                      0      //ȷ���Ƿ�ʹ�õ�����λ��
#define USING_RAIN                     0      //ȷ���Ƿ�ʹ��������
#define USING_ALARMER                  0      //ȷ���Ƿ�ʹ�����ܱ����� 

/* device id */
#define    CM_DBLF_ID   "11111111"      //�ر��ѷ��豸ID
#define    CM_QLF_ID   "22222222"      //ǽ�ѷ��豸ID
#define    CM_DMQX_ID   "333333333"      //������б�豸ID
#define    CM_MUD_ID   "44444444"      //��λ���豸ID
#define    CM_RAIN_ID   "55555555"      //�������豸ID
#define    CM_ANNU_ID   "66666666"      //���ܱ������豸ID

/* device type */
#define CM_EARTH_DEV            0xA1       //�ر��ѷ�
#define CM_WALL_DEV             0xA2       //ǽ�ѷ�
#define CM_ANGLE_DEV            0xA3       //���
#define CM_RAIN_DEV             0xA4       //����
#define CM_MUD_DEV              0xA5       // ��λ
#define CM_AUTO_DEV             0xA6        //����

/* data struct index */
#define  cm_head_que           0
#define  cm_count_que          1
#define  cm_spare_byte         2
#define  cm_data_type_que      3
#define  cm_sn_que             4
#define  cm_datalen1_que      12    //2���ֽ�
#define  cm_nb_type_que       14
#define  cm_datalen2_que      15   //1���ֽ�
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
    
//    float angle;      //����ϸ��
//	float wire;
	uint32_t cycle;
	uint32_t softversion; //16
	uint32_t sample;   //16
	float rain;        //16
	uint32_t autos;    //8
	uint32_t autolang;  //8
    float mud;         //16
    float dmqx_angle;   //������б
    float surface_ang;  //�ر�Ƕ���ֵ
    float surface_dis;  //�ر����߾���
    float wall_ang;     //ǽ�Ƕ���ֵ
    float wall_dis;     //ǽ������ֵ
};

/* struct main message value */
struct cm_mainmsg{
 
    uint8_t cmd; //�ڶ��������е�CMD,��������ӣ����������ϱ����ͺ�����/��ȡ����
	char *date;            //���� 7Byte
	char *time;            //ʱ�� 7Byte
	float dir_x;           //X�᷽��λ���� 4Byte
	float dir_y;           //Y�᷽��λ���� 4Byte
	float dir_z;           //Z�᷽��λ���� 4Byte
	float angle_x;         //X����� 4Byte
	float angle_y;          //Y����� 4Byte
	float angle_z;         //Z����� 4Byte
	float dist;            //���߳��� 4Byte
	uint8_t bat_per;        //�����ٷֱ� 1Byte
	float bat_vol;          //��ѹ 4Byte
	float gps_long;    //GPS����   4Byte
	float gps_lat;     //GPSγ��   4Byte
	float bd_long;     //BD����   4Byte
	float bd_lat;     //BDγ��   4Byte
	uint16_t signal;    //�ź� 2Byte
	float rain_n;       //����ǿ��   4Byte
    float mud_n;         //��λ 4Byte 
	uint8_t auto_s;       //���ܱ�������Դ���� 1byte
	uint8_t auto_lang;    //���ܱ������������� 1byte
    uint32_t pack_index;       //ota 4byte �ְ�����
	uint32_t pack_num;         //ota 4byte �ְ������趨һֱΪ1��
	uint8_t ota_lab;      //ota ���
	uint32_t ota_sum;     //ota  �ְ����ۼӺ�
};
typedef struct cm_mainmsg *cm_mainmsg_t;
/* data stuct */
struct cm_header{

	uint8_t version;
    uint8_t dev_type;  //��2�ֽڵı���count��Ϊ�豸����
    uint8_t spare;      //����һ���ֽ�
//    uint16_t count;
    uint8_t  msg_mode;
    char     *dev_id;     //8�ֽ�
    uint16_t  data_len; //2���ֽ�
    void     *user_data; //�ڶ�������	
};
typedef struct cm_header *cm_header_t;
/*������Ϣ�������ͣ����л������У�*/
enum cm_msg_id
{
	cm_msg_up = 0x00,
	cm_msg_upack = 0x01,
	cm_msg_down = 0x03,

};

/* list cmd values*/
enum cm_cmd_word
{
	cm_modulestatus_up = 0x01,       //�ϱ�ģ��״̬���ر�/ǽ�ѷ����ǡ�������б����ǡ���λ�����������ܱ�������
    cm_angle_read = 0x0F,                    //��ȡ�Ƕ�ƫ�Ƹ澯��ֵ���ر�/ǽ�ѷ����ǡ�������б����ǣ�
    cm_angle_up = 0x10,                      //�ϱ��Ƕ�ƫ�Ƹ澯��ֵ���ر�/ǽ�ѷ����ǡ�������б����ǣ�
    cm_angle_set = 0x11,                      //���ýǶ�ƫ�Ƹ澯��ֵ���ر�/ǽ�ѷ����ǡ�������б����ǣ�
    cm_dist_read = 0x15,                      //��ȡ����ƫ�Ƹ澯��ֵ���ر�/ǽ�ѷ����ǣ�
    cm_dist_up = 0x16,                      //�ϱ�����ƫ�Ƹ澯��ֵ���ر�/ǽ�ѷ����ǣ�
    cm_dist_set = 0x17,                      //��������ƫ�Ƹ澯��ֵ���ر�/ǽ�ѷ����ǣ�
    cm_cycle_read = 0x40,                     //��ȡ�������ڣ��ر�/ǽ�ѷ����ǡ�������б����ǡ���λ��
    cm_cycle_up = 0x41,                     //�ϱ��������ڣ��ر�/ǽ�ѷ����ǡ�������б����ǡ���λ��
    cm_cycle_set = 0x42,                     //�����������ڣ��ر�/ǽ�ѷ����ǡ�������б����ǡ���λ��
    cm_softversion_read = 0x43,              //  ��ȡ����汾 ���ر�/ǽ�ѷ����ǡ�������б����ǡ���λ��
    cm_softversion_up = 0x44,              //  �ϱ�����汾 ���ر�/ǽ�ѷ����ǡ�������б����ǡ���λ��
    cm_sample_read = 0x46,                 //  ��ȡ������� ���ر�/ǽ�ѷ����ǡ�������б����ǡ���λ��
    cm_sample_up = 0x47,                 //  �ϱ�������� ���ر�/ǽ�ѷ����ǡ�������б����ǡ���λ��
    cm_sample_set = 0x48,                //  ���ò������ ���ر�/ǽ�ѷ����ǡ�������б����ǡ���λ��
    cm_heartbeat_up = 0x37,               //  �ϱ������� ���ر�/ǽ�ѷ����ǡ�������б����ǡ���λ�����ܱ�������
    
    cm_ota_status_up = 0xF0,             // �豸�ϱ�Զ������״̬���ر�/ǽ�ѷ����ǡ�������б����ǡ���λ�����������ܱ�������
    cm_server_request_ota = 0xF1,        //  ���������豸�����������󣨵ر�/ǽ�ѷ����ǡ�������б����ǡ���λ�����������ܱ�������  
    cm_client_request_otadata = 0xF2,   //    �豸�������������������  ���ر�/ǽ�ѷ����ǡ�������б����ǡ���λ�����������ܱ�������
    cm_server_send_otadata = 0xF3,      //    ���������豸������������    ���ر�/ǽ�ѷ����ǡ�������б����ǡ���λ�����������ܱ�������
    
    cm_rain_cycle_read = 0x0C,         //��ȡ�������� (����)
    cm_rain_cycle_up = 0x0D,           // �ϱ��������� (����)
    cm_rain_cycle_set = 0x0E,           // ������������ (����)
    cm_rain_softversion_read = 0x0F,    //  ��ȡ����汾 (����)
    cm_rain_softversion_up = 0x10,    //  �ϱ�����汾 (����)
    cm_rain_sample_read = 0x12,    //  ��ȡ������� (����)
    cm_rain_sample_up = 0x13,    //  �ϱ�������� (����)
    cm_rain_sample_set = 0x14,    //  ���ò������ (����)
    cm_rain_threshold_read = 0x19,       //��ȡ�澯��ֵ(����)
    cm_rain_threshold_up = 0x1A,       //�ϱ��澯��ֵ (����)
    cm_rain_threshold_set = 0x1B,       //���ø澯��ֵ (����)
    
    cm_annu_read = 0x09,               // ��ȡ���ܱ��������������ܱ�������
    cm_annu_up = 0x0A,                //�ϱ����ܱ��������������ܱ�������
    cm_annu_set = 0x0B,                //�������ܱ��������������ܱ�������
    cm_annu_cycle_read = 0x0C,          //��ȡ�������� (���ܱ�����)
    cm_annu_cycle_up = 0x0D,          //�ϱ��������� (���ܱ�����)
    cm_annu_cycle_set = 0x0E,          //������������ (���ܱ�����)
    cm_annu_softversion_read = 0x0F,      //  ��ȡ����汾 (���ܱ�����)
    cm_annu_softversion_up = 0x10,      //  �ϱ�����汾 (���ܱ�����)
    cm_annu_sample_read = 0x12,    //  ��ȡ������� (���ܱ�����)
    cm_annu_sample_up = 0x13,    //  �ϱ�������� (���ܱ�����)
    cm_annu_sample_set = 0x14,    //  ���ò������ (���ܱ�����)
   
    cm_mud_threshold_read = 0x0F,       // ��ȡ��λ�澯��ֵ
    cm_mud_threshold_up = 0x10,        // �ϱ���λ�澯��ֵ
    cm_mud_threshold_set = 0x11,       // ������λ�澯��ֵ

};

uint16_t cm_encode(cm_header_t head,cm_mainmsg_t msg);
void cm_decode(uint16_t len,uint8_t *rev);
void cm_device_init(cm_header_t ptr,uint16_t length);

#endif



















