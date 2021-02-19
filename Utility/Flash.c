#include "main.h"
#include "Flash.h"


/****************************************************************
*Name:		ReadFlashNWord
*Function:	从内部Flash读取N个字数据（int）
*Input:		Offset：数据地址偏移；ReadBuf：字数据指针；Numb：读取数量
*Output:	无
*Author:  jhshen
*Date:		2020/07/26
*E-Mail:	jhshen163@163.com
*Other:		
****************************************************************/
void ReadFlashNWord(uint32_t Offset, int *ReadBuf, uint32_t Numb) 
{
    uint16_t i = 0;
    uint32_t Address;
    
    Address = (uint32_t) FLASH_PARA_ADDR + Offset; 
    for(i=0; i< Numb; i++) 
    {
       *(ReadBuf + i) = *(__IO uint32_t*) Address;  // read one int(word)
			 Address = Address + 4;   // 4 bytes
    }
}


/****************************************************************
*Name:		WriteFlashNWord，擦除一页！！
*Function:	写入FlashN个字数据（int）
*Input:		Offset：数据地址偏移；WriteBuf：字数据指针；Numb：写入数量
*Output:	无
*Author:  jhshen
*Date:		2020/07/26
*E-Mail:	jhshen163@163.com
*Other:		
****************************************************************/
void WriteFlashNWord(uint32_t Offset, int *Buf, uint32_t Numb)
{
  uint16_t i;  
	uint32_t Address;
    
    Address = (uint32_t) FLASH_PARA_ADDR + Offset; 

	//    SystemParameterStorage.Parameter = SystemParameter;
 
		    // 1. unLock FLASH
        HAL_FLASH_Unlock();
	
	      // 2. Erase FLASH
	      // Init FLASH_EraseInitTypeDef
	      FLASH_EraseInitTypeDef sf;
	      sf.TypeErase = FLASH_TYPEERASE_PAGES;
	      sf.PageAddress = Address;    // Page Address
	      sf.NbPages = 1;
	      // set PageError
	      uint32_t PageError = 0;
	      // Call erase function 
	      HAL_FLASHEx_Erase(&sf, &PageError);  // Erase one Page!!

	      // 3. write FLASH
				for (i=0; i<Numb; i++)
				  {
	          HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *(Buf+i) ); // write one int(word, 32 bits)
						Address = Address +4;   // 4 byte
					}	
	
    	  //4. Lock FLASH
        HAL_FLASH_Lock();
}

/****************************************************************
*Name:		WriteFlashCount，写总计数to Flash
*Function:	写入Flash 一个字数据（uint32 总计数），占用一页（2KB）。
           自动磨损控制：写满512个字（2KB）擦除一次。按1万次擦写寿命计算，可以计数512万次。
*Input:		写入数据
*Output:	无
*Author:  jhshen
*Date:		2021/01/17
*E-Mail:	jhshen163@163.com
*Other:		
****************************************************************/
void WriteFlashCount(uint32_t count)
{
	uint32_t Address, offset;
    
	  offset = (count % 512) *4 ;   // 4 byte
    Address = (uint32_t) FLASH_COUNT_ADDR + offset;  //  

 
		    // 1. unLock FLASH
        HAL_FLASH_Unlock();
	
	      // 2. Erase FLASH
	      // Init FLASH_EraseInitTypeDef
	      if (offset==0) 
					{
	          FLASH_EraseInitTypeDef sf;
	          sf.TypeErase = FLASH_TYPEERASE_PAGES;
	          sf.PageAddress = FLASH_COUNT_ADDR;    // Page Address
	          sf.NbPages = 1;
	          // set PageError
	          uint32_t PageError = 0;
	          // Call erase function 
	          HAL_FLASHEx_Erase(&sf, &PageError);  // Erase one Page 2KB !!
				  }

	      // 3. write FLASH
				  
	      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, count ); // write one int(word, 32 bits)
						
    	  //4. Lock FLASH
        HAL_FLASH_Lock();
}


uint32_t ReadFlashCount(void)
{
	uint32_t Address,  dt;
    
  Address = (uint32_t) FLASH_COUNT_ADDR + 2048-4;    // from TOP of 2KB
	do 
		{
      dt =  *(__IO uint32_t *)(Address);
		  Address = Address - 4;   // 4 bytes	
		}	while ( ( dt == 0xFFFFFFFF ) && (Address >= FLASH_COUNT_ADDR  ) );   // read one int(word)

	return(dt);
}
