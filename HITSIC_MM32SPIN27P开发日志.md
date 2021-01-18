## HITSIC_MM32SPIN27P开发日志

### Week1

#### 配置开发环境

使用工具：IAR8.32，官方开发套件和例程包

```
MM32_IAR_EWARM_pack_Ver1.50.zip
MM32SPIN2x_p_Lib_Samples_V1.11_SC.zip
IAR Embedded Workbench for ARM 8.32.1.zip
```

步骤：

##### 一、创建一个文件夹(HITSIC_MM32SPIN)

文件夹内部结构为

```
HITSIC_MM32SPIN_IAR          文件夹

LICENSE                      文件
```

LICENSE使社团的即可

HITSIC_MM32SPIN_IAR中创建几个文件夹

```
board
CMSIS
device
drivers
iar
source
startup
```

其中CMSIS使用社团MK66工程中的，iar中存放的为IAR工作区(workspace)、项目(project)和设置文件。

##### 二、创建IAR工程(HITSIC_MM32SPIN_IAR)

1.打开IAR8.32，创建新工作区,新项目，项目名称HITSIC_MM32SPIN_IAR，保存到iar中。

2.右键工程，添加group。

```
board
CMSIS
device
drivers
source
startup
```

##### 三、配置开发环境

解压MM32_IAR_EWARM_pack_Ver1.49.zip，运行MM32_EWARM_patch.exe。

该程序自动选择IAR，点击DriverPath即可，几秒之后，配置完成，点击close退出。如果此时打开了IAR，关掉IAR重进。

##### 四、移植文件

1.解压MM32SPIN2x_p_Lib_Samples_V1.11_SC.zip出现两个文件夹。

```
Device和Boards_MM32SPIN2x_p
```

startup：Device->MM32SPIN2xx_p_LIB->Source->IAR_StartAsm->startup_MM32SPIN2xx_p.s

device：Device->MM32SPIN2xx_p_LIB->Include->3个文件

​				Device->MM32SPIN2xx_p_LIB->Source->system_MM32SPIN2xx_p.c

drivers：Device->MM32SPIN2xx_p_LIB->HAL_lib->inc->全部文件

​				Device->MM32SPIN2xx_p_LIB->HAL_lib->src->全部文件

CMSIS：使用社团现有的

board：手动新建两个文件board.c,board.h

source：手动新建文件main.cpp，syscalls.c，isr.cpp。

2.将文件添加到工程中

打开各个文件夹，将里面的文件全选，拖到IAR工作区对应文件夹中

##### 五、设置IAR

右键工程->Options

General Options->Target->Processor variant->Device 选择MindMotion MM32SPIN27PS

General Options->Library Options 2->Heap selection 选择Advanced heap

Static Analysis->C_STAT 改为6 600 100

C/C++Compiler->Language 1 选c++

C/C++Compiler->Optimizations->Level 选None

C/C++Compiler->Encodings 全改为UTF-8，关掉with BOM

C/C++Compiler->List 选中Output list file，Diagnostics

C/C++Compiler->Preprocessor->Additional... 写

```
$PROJ_DIR$\..\board
$PROJ_DIR$\..\CMSIS
$PROJ_DIR$\..\device
$PROJ_DIR$\..\startup
$PROJ_DIR$\..\drivers
$PROJ_DIR$\..\source
```

C/C++Compiler->Preprocessor->Defined symbols 写

```
__IAR
OVERTIME=0
CLOCK=8000000
DEBUG
```

C/C++Compiler->Diagnostics

Supper...写

```
pa082,pe223,pe1665
```

Treat these as errors写

```
pe940,pe549
```

Linker->Optimizations-> 选中Merge

Linker->Advanced 关掉Allow C++

Linker->Encodings 全为UTF-8,关掉 with BOM

Debugger->Setup->Driver 选CMSIS DAP

##### 六、修改部分例程

1.在main.cpp中写

```
#include "MM32SPIN2xx_p.h"

int main(void)
{
  while(true)
  {
    
  }
  return 0;
}
```

运行后发现有BUG，打开MM32SPIN2xx_p.h，将186行至190行注释掉，写

```
#define TRUE  true
#define FALSE false
```

原因：采用c++编译器，不需要使用bool类型

2.打开system_MM32SPIN2xx_p.h

在

```
#define __SYSTEM_MM32SPIN2xx_p_H__
```

后写

```
#ifdef __cplusplus
extern "C"{
#endif
```

在最后一个

```
#endif
```

之前写

```
#ifdef __cplusplus
}
#endif
```

3.打开system_MM32SPIN2xx_p.c

在(约在149行)

```
/**
* @brief  Setup the microcontroller system
* Initialize the Embedded Flash Interface, the PLL and update the
* SystemCoreClock variable.
* @note   This function should be used only after reset.
* @param  None
* @retval None
*/
void SystemInit (void)
```

前写

```
#ifdef __cplusplus
extern "C"{
#endif
```

在程序最后

```
/*-------------------------(C) COPYRIGHT 2019 MindMotion ----------------------*/
```

之前写

```
#ifdef __cplusplus
}
#endif
```

4.保存编译，通过

### Week2

#### 学习数据手册

使用MM32SPIN27PS

闪存		128kB		封装		LQFP64

SRAM	 12kB		   CPU频率 96MHz



### Week3

#### 练习

首先观察单片机内部的总线，不同的外设通过不同名称的线与cpu相连，当调用这些外设的时候，要先开启对应总线的时钟，再针对外设进行初始化。

##### 一、GPIO口的使用

调用GPIO需要两个头文件，以下不知道来源的东西要到这两个文件及其源文件中找。

```
#include "HAL_gpio.h"
#include "HAL_rcc.h"
```

下面以开启开发板上蜂鸣器为例介绍（对应引脚为D6）

1.定义一个结构体

```
GPIO_InitTypeDef GPIO_InitStructure;
```

2.然后开启时钟

```
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
```

3.设置结构体中的变量(通用推挽输出)

```
GPIO_InitStructure1.GPIO_Pin=GPIO_PIN_6;
GPIO_InitStructure1.GPIO_Mode=GPIO_Mode_Out_PP;
GPIO_InitStructure1.GPIO_Speed=GPIO_Speed_50MHz;
```

4.初始化GPIO口

```
GPIO_Init(GPIOD,&GPIO_InitStructure);
```

5.对该GPIO口进行写操作

```
GPIO_WriteBit(GPIOD,GPIO_PIN_6,Bit_SET);//置1
delay();//延时，为了能听到响
GPIO_WriteBit(GPIOD,GPIO_PIN_6,Bit_RESET);//置0
```

6.GPIO其余操作详见HAL_gpio.h

##### 二、GPIO外部中断（EXTI）

要再包含以下头文件

```
#include "HAL_exti.h"
#include "HAL_misc.h"
#include "HAL_syscfg.h"
```

还是以打开蜂鸣器为例，这次使用中断，按键触发中断

对应的一些变量设置到对应的头文件找

```
//初始化一个外部中断，C1(KEY2)
GPIO_InitTypeDef GPIO_InitStructure;//GPIO
EXTI_InitTypeDef EXTI_InitStructure;//外部中断
NVIC_InitTypeDef NVIC_InitStructure;//中断管理器
//开总线时钟
RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//详见手册要求
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOC, &GPIO_InitStructure);

SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);//选择C1作为外部中断线路
EXTI_InitStructure.EXTI_Line = EXTI_Line1;
EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
EXTI_InitStructure.EXTI_LineCmd = ENABLE;
EXTI_Init(&EXTI_InitStructure);

NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;     //IRQn type里找
NVIC_InitStructure.NVIC_IRQChannelPriority = 3;        //优先级
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	 //IRQ通道使能
NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化NVIC寄存器
```

对应的中断服务函数到startup文件中找

注意在中断服务程序中要手动将中断标志位清0

为了正常运行，中断服务函数前后要加上

```
#ifdef __cplusplus
extern "C"{
#endif

/*code*/

#ifdef __cplusplus
}
#endif
```



```
#ifdef __cplusplus
extern "C"{
#endif
void EXTI0_1_IRQHandler(void)//名字看startup
{
  EXTI_ClearFlag(EXTI_Line1);//清除中断标志位
  GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_SET);//打开蜂鸣器
  delay(1000);
  GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_RESET);//关闭
}
#ifdef __cplusplus
}
#endif
```

##### 三、定时器中断（TIM）

再包含头文件

```
#include "HAL_tim.h"
```

以蜂鸣器为例，这次让它定时响。

以TIM1（APB2）为例

```
//定时器中断TIM1为例在APB2上
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//开时钟APB2

TIM_TimeBaseInitStructure.TIM_Prescaler = 10000;    //预分频，总线上的时钟频率除此数得计数频率
TIM_TimeBaseInitStructure.TIM_Period = 10000;       //计多少个数
TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;    //时钟分频，没什么影响
TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

NVIC_InitTypeDef NVIC_InitStructure;
NVIC_InitStructure.NVIC_IRQChannel=TIM1_BRK_UP_TRG_COM_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPriority=3;
NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
NVIC_Init(&NVIC_InitStructure);

TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);//相应中断，此处为计数器溢出更新引起中断
TIM_Cmd(TIM1, ENABLE);//计数器使能
```

到startup中找到对应的中断服务函数（更新引起的中断）

```
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM1, TIM_IT_Update);//手动清
  GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_SET);
  delay(100);
  GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_RESET);
}
```

##### 四、系统时钟

核心板上应用了8MHz无源晶振(HSE)，在程序中设定频率。

时钟图见数据手册P7

外部时钟源接在OSC_OUT和OSC_IN之间，通过PLL锁相环输出SYSCLK，接下来，SYSCLK通过AHB，APB的分频，为各个外设提供时钟信号。此处程序便是设置分频的。

此处编写一个子程序。**推荐倍数为12**

```
void HSE_SetSysClk(uint32_t RCC_PLLMul_x)//传入参数为倍频
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
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_x);//PLL输入时钟，倍频
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
    //启动失败，添加错误处理代码
    /*code*/
  }
}
```

此函数添加在主程序起始部分。

##### 五、I2C

硬件I2C没有调出来，所以使用软件IO口模拟。

自己包装两个文件

```
#include "i2c.h"
#include "i2c.c"
```

其中应用到了延时函数，包含在

```
#include "delay.h"
#include "delay.c"
```

软件I2C供用户使用的函数有

```
//for user
void SOFT_I2C_Init(void);
void SOFT_I2C_Read(uint8_t SlaveAddr, uint8_t reg, uint8_t* rxbuff, uint8_t len);
void SOFT_I2C_Write(uint8_t SlaveAddr, uint8_t reg, uint8_t* txbuff, uint8_t len);
```

用户可修改的地方为软件I2C的引脚

//可修改

```
#define SOFT_I2C_SCL GPIO_Pin_8
#define SOFT_I2C_SDA GPIO_Pin_9
#define GPIO_SOFT_I2C GPIOB
#define RCC_SOFT_I2C_GPIO RCC_AHBPeriph_GPIOB
```

智能车制作过程中I2C应用在陀螺仪数据传递中，所以下面以MPU6050数据接收为例介绍

MPU6050相关文件为

```
#include "I2C_MPU6050.h"
#include "I2C_MPU6050.c"
```

用户使用的函数为

```
void MPU6050_Init(void);
void inv_mpu6050_poll_sensor_data_reg(inv_mpu6050_t* s);
```

有关MPU6050配置的函数可自行查看上面的两个文件

在文件中已经定义一个结构体icm1，这个结构体中包含传回来的参数

实际使用的时候使用定时中断14接收数据

```
//陀螺仪
void TIM14_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
  inv_mpu6050_poll_sensor_data_reg(&icm1);
  //a=icm1.rawdata.ax;
}
```

