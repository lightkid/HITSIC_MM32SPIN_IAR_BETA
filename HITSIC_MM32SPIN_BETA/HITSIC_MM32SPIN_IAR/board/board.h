#ifndef _BOARD_H_
#define _BOARD_H_

//#include "dtype.h"
//#include "HAL_adc.h"
//#include "HAL_comp.h"
//#include "HAL_conf.h"
//#include "HAL_crc.h"
//#include "HAL_div.h"
//#include "HAL_dma.h"
#include "HAL_exti.h"
#include "HAL_flash.h"
#include "flash.h"
#include "HAL_gpio.h"
//#include "HAL_i2c.h"
//#include "HAL_iwdg.h"
#include "HAL_misc.h"
//#include "HAL_op.h"
//#include "HAL_pwr.h"
#include "HAL_rcc.h"
//#include "HAL_spi.h"
#include "HAL_syscfg.h"
#include "HAL_tim.h"
#include "HAL_uart.h"
#include "uart.h"
//#include "HAL_wwdg.h"
#include "HAL_device.h"
#include "I2C_MPU6050.h"
#include "i2c.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
//#include <string.h> 
///*! @brief Construct a status code value from a group and code number. */
//#define MAKE_STATUS(group, code) ((((group)*100) + (code)))
//
///*! @brief Status group numbers. */
//enum _status_groups
//{
//    kStatusGroup_Generic = 0,                 /*!< Group number for generic status codes. */
//    kStatusGroup_FLASH = 1,                   /*!< Group number for FLASH status codes. */
//    kStatusGroup_SPI = 6,                    /*!< Group number for SPI status codes. */
//    kStatusGroup_UART = 10,                   /*!< Group number for UART status codes. */
//    kStatusGroup_I2C = 11,                    /*!< Group number for UART status codes. */
//    
//};
//
///*! @brief Generic status return codes. */
//enum _generic_status
//{
//    kStatus_Success = MAKE_STATUS(kStatusGroup_Generic, 0),
//    kStatus_Fail = MAKE_STATUS(kStatusGroup_Generic, 1),
//    kStatus_ReadOnly = MAKE_STATUS(kStatusGroup_Generic, 2),
//    kStatus_OutOfRange = MAKE_STATUS(kStatusGroup_Generic, 3),
//    kStatus_InvalidArgument = MAKE_STATUS(kStatusGroup_Generic, 4),
//    kStatus_Timeout = MAKE_STATUS(kStatusGroup_Generic, 5),
//    kStatus_NoTransferInProgress = MAKE_STATUS(kStatusGroup_Generic, 6),
//};
//
///*! @brief Type used for all status and error return values. */
//typedef int32_t status_t;


void HSE_SetSysClk(uint32_t RCC_PLLMul_x);

#endif