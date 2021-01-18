#include "i2c.h"


void SOFT_I2C_Init(void)
{
  RCC_AHBPeriphClockCmd(RCC_SOFT_I2C_GPIO, ENABLE);
  SOFT_I2C_SCL_OUT();
  SOFT_I2C_SDA_OUT();
  SOFT_I2C_SCL_H;
  SOFT_I2C_SDA_H;
}

void SOFT_I2C_SCL_OUT(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	
  
  GPIO_InitStructure.GPIO_Pin=SOFT_I2C_SCL;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
  GPIO_Init(GPIO_SOFT_I2C,&GPIO_InitStructure);
}

void SOFT_I2C_SDA_OUT(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	
  
  GPIO_InitStructure.GPIO_Pin=SOFT_I2C_SDA;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
  GPIO_Init(GPIO_SOFT_I2C,&GPIO_InitStructure);
}

void SOFT_I2C_SDA_IN(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	
  GPIO_InitStructure.GPIO_Pin=SOFT_I2C_SDA;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
  GPIO_Init(GPIO_SOFT_I2C,&GPIO_InitStructure);
}

//������ʼ�ź�
void SOFT_I2C_Start(void)
{
  SOFT_I2C_SDA_OUT();
  SOFT_I2C_SDA_H;
  SOFT_I2C_SCL_H;
  delay_us(5);
  SOFT_I2C_SDA_L;
  delay_us(3);
  SOFT_I2C_SCL_L;
}

//����ֹͣ�ź�
void SOFT_I2C_Stop(void)
{
  SOFT_I2C_SDA_OUT();
  SOFT_I2C_SCL_L;
  SOFT_I2C_SDA_L;
  delay_us(3);
  SOFT_I2C_SCL_H;
  delay_us(3);
  SOFT_I2C_SDA_H;
  delay_us(1);
}

//��������Ӧ���ź�ACK
void SOFT_I2C_Ack(void)
{
  SOFT_I2C_SCL_L;
  SOFT_I2C_SDA_OUT();
  SOFT_I2C_SDA_L;
  delay_us(3);
  SOFT_I2C_SCL_H;
  delay_us(3);
  SOFT_I2C_SCL_L;
}

//����������Ӧ���ź�NACK
void SOFT_I2C_NAck(void)
{
  SOFT_I2C_SCL_L;
  SOFT_I2C_SDA_OUT();
  SOFT_I2C_SDA_H;
  delay_us(3);
  SOFT_I2C_SCL_H;
  delay_us(3);
  SOFT_I2C_SCL_L;
}
//�ȴ��ӻ�Ӧ���ź�
//����ֵ��1 ����Ӧ��ʧ��
//        0 ����Ӧ��ɹ�
uint8_t SOFT_I2C_Wait_Ack(void)
{
  uint8_t tempTime=0;
  SOFT_I2C_SDA_IN();
  SOFT_I2C_SDA_H;
  delay_us(1);
  SOFT_I2C_SCL_H;
  delay_us(1);
  while(GPIO_ReadInputDataBit(GPIO_SOFT_I2C,SOFT_I2C_SDA))
  {
    tempTime++;
    if(tempTime > I2C_WAIT_TIMEOUT)
    {
      SOFT_I2C_Stop();
      return 1;
    }	 
  }
  SOFT_I2C_SCL_L;
  return 0;
}
//I2C ����һ���ֽ�
void SOFT_I2C_Send_Byte(uint8_t txd)
{
  uint8_t i=0;
  SOFT_I2C_SDA_OUT();
  SOFT_I2C_SCL_L;//����ʱ�ӿ�ʼ���ݴ���
  for(i = 0; i < 8; i++)
  {
    if(txd & 0x80) //0x80  1000 0000
      SOFT_I2C_SDA_H;
    else
      SOFT_I2C_SDA_L;
    SOFT_I2C_SCL_H;
    delay_us(2); //��������
    txd <<= 1;
    SOFT_I2C_SCL_L;
    delay_us(2);
  }
  delay_us(1);
}

//I2C ��ȡһ���ֽ�

uint8_t SOFT_I2C_Read_Byte(uint8_t ack)
{
  uint8_t i = 0, receive = 0;
  
  SOFT_I2C_SDA_IN();
  for(i = 0; i < 8; i++)
  {
    SOFT_I2C_SCL_L;
    delay_us(3);
    SOFT_I2C_SCL_H;
    receive<<=1;
    if(GPIO_ReadInputDataBit(GPIO_SOFT_I2C,SOFT_I2C_SDA))
      receive++;
    delay_us(1);	
  }
  if(ack == 0)
    SOFT_I2C_NAck();
  else
    SOFT_I2C_Ack();
  return receive;
}


void SOFT_I2C_Read(uint8_t SlaveAddress, uint8_t reg, uint8_t* data, uint8_t len)
{
  uint8_t i = 0;
  SOFT_I2C_Start();
  SOFT_I2C_Send_Byte(SlaveAddress);//������ַ+���ݵ�ַ
  if(SOFT_I2C_Wait_Ack())
  {
    SOFT_I2C_Stop();
    //return 1;//�ӻ���ַд��ʧ��
  }
  SOFT_I2C_Send_Byte(reg);//˫�ֽ������ݵ�ַ��λ		
  SOFT_I2C_Wait_Ack();
  SOFT_I2C_Start();
  SOFT_I2C_Send_Byte(SlaveAddress + 0x01);
  SOFT_I2C_Wait_Ack();
  for(i = 0; i < len; i++)
  {
    if(i != len - 1)
      *(data + i) = SOFT_I2C_Read_Byte(1); //  1   ���� ACK
    else
      *(data + i) = SOFT_I2C_Read_Byte(0); //  0   ���� NACK
  }
  SOFT_I2C_Stop();		
}

void SOFT_I2C_Write(uint8_t SlaveAddress,uint8_t reg,uint8_t* data, uint8_t len)
{
  uint8_t i = 0;
  SOFT_I2C_Start();
  SOFT_I2C_Send_Byte(SlaveAddress);//������ַ+���ݵ�ַ
  if(SOFT_I2C_Wait_Ack())
  {
    SOFT_I2C_Stop();
    //return 1;//�ӻ���ַд��ʧ��
  }
  SOFT_I2C_Send_Byte(reg);//˫�ֽ������ݵ�ַ��λ		
  SOFT_I2C_Wait_Ack();
  for(i = 0; i < len; i++)
  {
    SOFT_I2C_Send_Byte(*(data+i));
    if(SOFT_I2C_Wait_Ack())
    {
      SOFT_I2C_Stop();
      //return 1;//д��ʧ��
    }
  }
  SOFT_I2C_Stop();
}
