#include "devtype.h"
void getint(uint32_t *dat,uint8_t *src)
{
	uint8_t i;
	uint8_t *ptr;
	ptr=(uint8_t*)dat;
	for(i=0;i<4;i++)
	{
		*src++ = *ptr++;
	}
}

void getfloat(float *f, uint8_t *src)
{
	uint8_t i;
	uint8_t *ptr;
	ptr=(uint8_t*)f;
	for(i=0;i<4;i++)
	{
		*src++ = *ptr++;
	}
}



void strcopy(uint8_t *dst, uint8_t *src, uint8_t len)
{
	while(len--)
	{
		*dst++ = *src++;
	}
	*dst='\0';
}


void Hex2Str(char* pSrc, char* pDst, unsigned int nSrcLength)
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

void str2hex(char *src,uint8_t *dst,uint16_t len)
{
	uint16_t i;
	char *ptr=src;
	uint8_t h,l;
	for(i=0;i<2*len;i++)
	{
		if(0x29 < *ptr && *ptr < 0x3a)
		{
			h = *ptr- '0';
		}
		else if(0x40 < *ptr && *ptr < 0x47)
		{
			h = *ptr- 'A'+10;
		}
        else if (96< *ptr && *ptr <123)
        {
            h = *ptr- 'a'+10;
        }
		ptr++;
		
		if(0x29 < *ptr && *ptr < 0x3a)
		{
			l = *ptr- '0';
		}
		else if(0x40 < *ptr && *ptr < 0x47)
		{
			l = *ptr- 'A' + 10;
		}
        else if (96< *ptr && *ptr <123)
        {
//            h = *ptr- 'a'+10;
              l = *ptr- 'a'+10;
        }
		ptr++;
		
		*dst++ = h<<4 | (l & 0x0f);
		i++;
	}
}






