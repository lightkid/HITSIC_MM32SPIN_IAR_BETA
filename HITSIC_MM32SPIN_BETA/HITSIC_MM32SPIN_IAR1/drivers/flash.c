#include "flash.h"



//У��FLASH��Ӧҳ��������
uint8_t Flash_Check (FLASH_SEC_enum sector_no, FLASH_PAGE_enum page_no)
{
    uint16_t temp_loop;
    uint32_t flash_addr = ((FLASH_BASE_ADDR+FLASH_SECTION_SIZE*sector_no+FLASH_PAGE_SIZE*page_no)); // ��ȡ��ǰ Flash ��ַ

    for(temp_loop = 0; temp_loop < FLASH_PAGE_SIZE; temp_loop+=4)      // ѭ����ȡ Flash
    {
        if( (*(__IO uint32_t*) (flash_addr+temp_loop)) != 0xFFFFFFFF ) //  0xFFFFFFFFΪ��
            return 1;
    }
    return 0;
}
//����һҳ����
uint8_t Flash_Erase_Page (FLASH_SEC_enum sector_no, FLASH_PAGE_enum page_no)
{
    static volatile FLASH_Status gFlashStatus = FLASH_COMPLETE;
    uint32_t flash_addr = ((FLASH_BASE_ADDR+FLASH_SECTION_SIZE*sector_no+FLASH_PAGE_SIZE*page_no));     // ��ǰ Flash ��ַ

    FLASH_Unlock();                                                                                     // ���� Flash
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);                           // ���������־
    gFlashStatus = FLASH_ErasePage(flash_addr);                                                         // ����
    FLASH_ClearFlag(FLASH_FLAG_EOP );                                                                   // ���������־
    FLASH_Lock();                                                                                       // ���� Flash
    if(gFlashStatus != FLASH_COMPLETE)                                                                  // �жϲ����Ƿ�ɹ�
        return 1;
    return 0;
}
//��ȡһҳ����
void Flash_Page_Read (FLASH_SEC_enum sector_no, FLASH_PAGE_enum page_no, uint32_t *buf, uint16_t len)
{
    uint16_t temp_loop = 0;
    uint32_t flash_addr = 0;
    flash_addr = ((FLASH_BASE_ADDR+FLASH_SECTION_SIZE*sector_no+FLASH_PAGE_SIZE*page_no));            // ���㵱ǰ Flash��ַ
    //flash_addr = ((uint32_t)0x0800F000);                                                              // ��ʼ��ַ

    for(temp_loop = 0; temp_loop < len; temp_loop++)                                                  // ����ָ�����ȶ�ȡ
    {
        *buf++ = *(__IO uint32_t*)(flash_addr+temp_loop*4);                                           // ѭ����ȡ Flash ��ֵ
    }
}
//д��һҳ����
uint8_t Flash_Page_Write (FLASH_SEC_enum sector_no, FLASH_PAGE_enum page_no, const uint32_t *buf, uint16_t len)
{
    static volatile FLASH_Status gFlashStatus = FLASH_COMPLETE;
    uint32_t flash_addr = 0;

    flash_addr = ((FLASH_BASE_ADDR+FLASH_SECTION_SIZE*sector_no+FLASH_PAGE_SIZE*page_no));            // ���㵱ǰ Flash��ַ

    if(Flash_Check(sector_no, page_no))                                                               // �ж��Ƿ������� ����������ı��� ��ֹ����û������д��
        Flash_Erase_Page(sector_no, page_no);                                                         // ������һҳ

    FLASH_Unlock();                                                                                   // ���� Flash
    while(len--)                                                                                      // ���ݳ���
    {
        gFlashStatus = FLASH_ProgramWord(flash_addr, *buf++);                                         // ���� 32bit д������
        if(gFlashStatus != FLASH_COMPLETE)                                                            // ����ȷ�ϲ����Ƿ�ɹ�
            return 1;
        flash_addr += 4;                                                                              // ��ַ����
    }
    FLASH_Lock();                                                                                     // ���� Flash
    return 0;
}