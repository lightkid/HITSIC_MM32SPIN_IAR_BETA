#ifndef __FLASH_H_
#define __FLASH_H_

#include "board.h"

#define FLASH_BASE_ADDR             (0x08000000)   //FALSH起始地址
#define FLASH_PAGE_SIZE             1024           //1K BYTE
#define FLASH_SECTION_SIZE          (FLASH_PAGE_SIZE*4) //4K BYTE
////以下的可以根据需要更改
//#define PAGE_WRITE_START_ADDR  ((uint32_t)0x0800F000) /* Start from 60K */
//#define PAGE_WRITE_END_ADDR    ((uint32_t)0x08010000) /* End at 63K */
//
//#define FLASH_PAGES_TO_BE_PROTECTED FLASH_WRProt_Pages60to63

//选择哪块，一块4k
typedef enum
{
    FLASH_SECTION_00,
    FLASH_SECTION_01,
    FLASH_SECTION_02,
    FLASH_SECTION_03,
    FLASH_SECTION_04,
    FLASH_SECTION_05,
    FLASH_SECTION_06,
    FLASH_SECTION_07,
    FLASH_SECTION_08,
    FLASH_SECTION_09,
    FLASH_SECTION_10,
    FLASH_SECTION_11,
    FLASH_SECTION_12,
    FLASH_SECTION_13,
    FLASH_SECTION_14,
    FLASH_SECTION_15,
}FLASH_SEC_enum;
//一块4页
typedef enum
{
    FLASH_PAGE_0,
    FLASH_PAGE_1,
    FLASH_PAGE_2,
    FLASH_PAGE_3,
}FLASH_PAGE_enum;

uint8_t Flash_Check(FLASH_SEC_enum sector_num, FLASH_PAGE_enum page_num);
uint8_t Flash_Erase_Page(FLASH_SEC_enum sector_num, FLASH_PAGE_enum page_num);
void Flash_Page_Read (FLASH_SEC_enum sector_num, FLASH_PAGE_enum page_num, uint32_t *buf, uint16_t len);
uint8_t Flash_Page_Write(FLASH_SEC_enum sector_num, FLASH_PAGE_enum page_num, const uint32_t *buf, uint16_t len);

#endif