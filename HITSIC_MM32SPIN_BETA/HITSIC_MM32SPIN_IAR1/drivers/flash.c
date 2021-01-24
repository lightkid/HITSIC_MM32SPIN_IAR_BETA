#include "flash.h"



//校验FLASH对应页有无数据
uint8_t Flash_Check (FLASH_SEC_enum sector_no, FLASH_PAGE_enum page_no)
{
    uint16_t temp_loop;
    uint32_t flash_addr = ((FLASH_BASE_ADDR+FLASH_SECTION_SIZE*sector_no+FLASH_PAGE_SIZE*page_no)); // 提取当前 Flash 地址

    for(temp_loop = 0; temp_loop < FLASH_PAGE_SIZE; temp_loop+=4)      // 循环读取 Flash
    {
        if( (*(__IO uint32_t*) (flash_addr+temp_loop)) != 0xFFFFFFFF ) //  0xFFFFFFFF为空
            return 1;
    }
    return 0;
}
//擦除一页数据
uint8_t Flash_Erase_Page (FLASH_SEC_enum sector_no, FLASH_PAGE_enum page_no)
{
    static volatile FLASH_Status gFlashStatus = FLASH_COMPLETE;
    uint32_t flash_addr = ((FLASH_BASE_ADDR+FLASH_SECTION_SIZE*sector_no+FLASH_PAGE_SIZE*page_no));     // 当前 Flash 地址

    FLASH_Unlock();                                                                                     // 解锁 Flash
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);                           // 清除操作标志
    gFlashStatus = FLASH_ErasePage(flash_addr);                                                         // 擦除
    FLASH_ClearFlag(FLASH_FLAG_EOP );                                                                   // 清楚操作标志
    FLASH_Lock();                                                                                       // 锁定 Flash
    if(gFlashStatus != FLASH_COMPLETE)                                                                  // 判断操作是否成功
        return 1;
    return 0;
}
//读取一页数据
void Flash_Page_Read (FLASH_SEC_enum sector_no, FLASH_PAGE_enum page_no, uint32_t *buf, uint16_t len)
{
    uint16_t temp_loop = 0;
    uint32_t flash_addr = 0;
    flash_addr = ((FLASH_BASE_ADDR+FLASH_SECTION_SIZE*sector_no+FLASH_PAGE_SIZE*page_no));            // 计算当前 Flash地址
    //flash_addr = ((uint32_t)0x0800F000);                                                              // 起始地址

    for(temp_loop = 0; temp_loop < len; temp_loop++)                                                  // 根据指定长度读取
    {
        *buf++ = *(__IO uint32_t*)(flash_addr+temp_loop*4);                                           // 循环读取 Flash 的值
    }
}
//写入一页数据
uint8_t Flash_Page_Write (FLASH_SEC_enum sector_no, FLASH_PAGE_enum page_no, const uint32_t *buf, uint16_t len)
{
    static volatile FLASH_Status gFlashStatus = FLASH_COMPLETE;
    uint32_t flash_addr = 0;

    flash_addr = ((FLASH_BASE_ADDR+FLASH_SECTION_SIZE*sector_no+FLASH_PAGE_SIZE*page_no));            // 计算当前 Flash地址

    if(Flash_Check(sector_no, page_no))                                                               // 判断是否有数据 这里是冗余的保护 防止有人没擦除就写入
        Flash_Erase_Page(sector_no, page_no);                                                         // 擦除这一页

    FLASH_Unlock();                                                                                   // 解锁 Flash
    while(len--)                                                                                      // 根据长度
    {
        gFlashStatus = FLASH_ProgramWord(flash_addr, *buf++);                                         // 按字 32bit 写入数据
        if(gFlashStatus != FLASH_COMPLETE)                                                            // 反复确认操作是否成功
            return 1;
        flash_addr += 4;                                                                              // 地址自增
    }
    FLASH_Lock();                                                                                     // 锁定 Flash
    return 0;
}