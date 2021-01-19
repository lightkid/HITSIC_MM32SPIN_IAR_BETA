#ifndef __I2C_H_
#define __I2C_H_

#include "board.h"

//softi2c

///*! @brief Timeout times for waiting flag. */
#define I2C_WAIT_TIMEOUT 200U /* Define to zero means keep waiting until the flag is assert/deassert. */
//
///*! @brief  I2C status return codes. */
//enum _i2c_status
//{
//    kStatus_I2C_Busy            = MAKE_STATUS(kStatusGroup_I2C, 0), /*!< I2C is busy with current transfer. */
//    kStatus_I2C_Idle            = MAKE_STATUS(kStatusGroup_I2C, 1), /*!< Bus is Idle. */
//    kStatus_I2C_Nak             = MAKE_STATUS(kStatusGroup_I2C, 2), /*!< NAK received during transfer. */
//    kStatus_I2C_ArbitrationLost = MAKE_STATUS(kStatusGroup_I2C, 3), /*!< Arbitration lost during transfer. */
//    kStatus_I2C_Timeout         = MAKE_STATUS(kStatusGroup_I2C, 4), /*!< Timeout poling status flags. */
//    kStatus_I2C_Addr_Nak        = MAKE_STATUS(kStatusGroup_I2C, 5), /*!< NAK received during the address probe. */
//};
//¿ÉÐÞ¸Ä
#define SOFT_I2C_SCL GPIO_Pin_8
#define SOFT_I2C_SDA GPIO_Pin_9
#define GPIO_SOFT_I2C GPIOB
#define RCC_SOFT_I2C_GPIO RCC_AHBPeriph_GPIOB

#define SOFT_I2C_SCL_H GPIO_SetBits(GPIO_SOFT_I2C,SOFT_I2C_SCL)
#define SOFT_I2C_SCL_L GPIO_ResetBits(GPIO_SOFT_I2C,SOFT_I2C_SCL)

#define SOFT_I2C_SDA_H GPIO_SetBits(GPIO_SOFT_I2C,SOFT_I2C_SDA)
#define SOFT_I2C_SDA_L GPIO_ResetBits(GPIO_SOFT_I2C,SOFT_I2C_SDA)



//for user
void SOFT_I2C_Init(void);
void SOFT_I2C_Read(uint8_t SlaveAddr, uint8_t reg, uint8_t* rxbuff, uint8_t len);
void SOFT_I2C_Write(uint8_t SlaveAddr, uint8_t reg, uint8_t* txbuff, uint8_t len);

void SOFT_I2C_SCL_OUT(void);
void SOFT_I2C_SDA_OUT(void);
void SOFT_I2C_SDA_IN(void);
void SOFT_I2C_Start(void);
void SOFT_I2C_Stop(void);
void SOFT_I2C_Ack(void);
void SOFT_I2C_NAck(void);
uint8_t   SOFT_I2C_Wait_Ack(void);
void SOFT_I2C_Send_Byte(uint8_t txd);
uint8_t   SOFT_I2C_Read_Byte(uint8_t ack);


#endif