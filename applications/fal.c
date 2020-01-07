#include "stm32l4xx_hal.h"

#define LASTPAGE 0x0803f800      //2k
#define LASTPAGE_END 0x0803ffff

static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;
  
  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
  	/* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
  	/* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }
  
  return bank;
}

static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;
  
  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }
  
  return page;
}

static FLASH_EraseInitTypeDef EraseInitStruct;
int fal_write(uint32_t Address,uint32_t *buf_addr,uint32_t length)
{
	HAL_StatusTypeDef err;
	Address = LASTPAGE;    //Ð´flashÊ×µØÖ·
	uint32_t banknumber = 0,first_page =0,PAGEError = 0;
	uint64_t *src_addr = (uint64_t*)buf_addr;
	uint64_t wdata;

	
	banknumber = GetBank(Address);
	first_page = GetPage(Address);
	 
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR); 
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks     = banknumber;
	EraseInitStruct.Page      = first_page;
	EraseInitStruct.NbPages   = 1; 
	err = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);       //²Á³ý
	if(err!=HAL_OK)
	{
		rt_kprintf("flash erase error\n");
	}
	do{

		wdata = *src_addr;
		err = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, (uint64_t)wdata);    //FLASH_TYPEPROGRAM_FAST Ð´32*64bit×Ö½Ú FLASH_TYPEPROGRAM_DOUBLEWORD
		if(err!=HAL_OK)
		{
			rt_kprintf("flash write error\n");
		}
		Address =  Address + sizeof(uint64_t);
		src_addr = src_addr + 1;
	}while(length--);
	
	HAL_FLASH_Lock();
	return 0;
}	

void fal_read(uint32_t Address,uint32_t *buf,uint32_t length)
{
	Address = LASTPAGE;
	uint8_t n = length / 4;
	__IO uint32_t *src_addr = (__IO uint32_t*)Address;
	__disable_irq();
	do{
		*buf = *src_addr;
		buf++;
		src_addr++;
	}while(length--);
	__enable_irq();
}
















