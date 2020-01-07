#ifndef __DEVTYPE_H__
#define __DEVTYPE_H__
#include "stm32l476xx.h"
#include <rtthread.h>

#define  CGATT 1<<0               //事件的标志值
#define  CEREG 1<<1
//#define  QLWEVTIND 1<<2
//#define  MSG 1<<3
#define  EVENT 1<<4
#define  Observe_msgid 1<<5
#define  Discover_msgid 1<<6
//#define  DREG_EVENT 1<<7
#define  ota_event 1<<8
#define  cfun      1<<9
//#define  ERROR_50  1<<10
#define  NBNET 1<<11
#define  GPS 1<<12
#define  FAIL_REG_NET 1<<13
#define  CME50 1<<14


extern uint8_t cm_senddata[512];
extern struct rt_event nb_event;
void getint(uint32_t *dat,uint8_t *src);
void getfloat(float *f, uint8_t *src);
void strcopy(uint8_t *dst, uint8_t *src, uint8_t len);
void Hex2Str(char* pSrc, char* pDst, unsigned int nSrcLength);
void str2hex(char *src,uint8_t *dst,uint16_t len);
uint8_t crc8_table(uint8_t *ptr, uint8_t len);

void refreshdata(void);
void nb_send(char* msg,uint16_t len);

#endif
