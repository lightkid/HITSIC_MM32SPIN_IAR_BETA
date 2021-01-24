#include "board.h"

void HSE_SetSysClk(uint32_t RCC_PLLMul_x)
{
  ErrorStatus HSEStatus;
  RCC_DeInit();//RCC时钟复位
  RCC_HSEConfig(RCC_HSE_ON);//开启外部晶振
  HSEStatus = RCC_WaitForHSEStartUp();//等待获取HSE启动状态
  //HSE启动成功
  if(HSEStatus == SUCCESS)
  {
    //FLASH延时
    //???
    //配置总线分频因子
    RCC_HCLKConfig(RCC_SYSCLK_Div1);//AHB
    RCC_PCLK1Config(RCC_HCLK_Div2);//APB1
    RCC_PCLK2Config(RCC_HCLK_Div1);//APB2
    //PLL锁相环的输入源和倍频因子，使能PLL
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_x);
    RCC_PLLCmd(ENABLE);
    //等待PLL准备就绪
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    //选择PLLCLK作为系统时钟
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    //等待系统时钟切换成功并稳定
    while (RCC_GetSYSCLKSource() != 0x08);//0x08代表系统使用PLLCLK

  }
  else
  {
    //启动失败
  }
}
