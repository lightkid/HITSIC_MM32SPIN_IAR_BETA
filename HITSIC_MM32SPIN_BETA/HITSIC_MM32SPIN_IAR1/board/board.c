#include "board.h"

void HSE_SetSysClk(uint32_t RCC_PLLMul_x)
{
  ErrorStatus HSEStatus;
  RCC_DeInit();//RCCʱ�Ӹ�λ
  RCC_HSEConfig(RCC_HSE_ON);//�����ⲿ����
  HSEStatus = RCC_WaitForHSEStartUp();//�ȴ���ȡHSE����״̬
  //HSE�����ɹ�
  if(HSEStatus == SUCCESS)
  {
    //FLASH��ʱ
    //???
    //�������߷�Ƶ����
    RCC_HCLKConfig(RCC_SYSCLK_Div1);//AHB
    RCC_PCLK1Config(RCC_HCLK_Div2);//APB1
    RCC_PCLK2Config(RCC_HCLK_Div1);//APB2
    //PLL���໷������Դ�ͱ�Ƶ���ӣ�ʹ��PLL
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_x);
    RCC_PLLCmd(ENABLE);
    //�ȴ�PLL׼������
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    //ѡ��PLLCLK��Ϊϵͳʱ��
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    //�ȴ�ϵͳʱ���л��ɹ����ȶ�
    while (RCC_GetSYSCLKSource() != 0x08);//0x08����ϵͳʹ��PLLCLK

  }
  else
  {
    //����ʧ��
  }
}
