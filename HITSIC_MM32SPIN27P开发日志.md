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