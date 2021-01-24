#include "oled.h"

void OLED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHBPeriphClockCmd(OLED_RCC, ENABLE);//main中应先开全部时钟，此条便可注释
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15|GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(OLED_GPIO, &GPIO_InitStructure);  
  
  OLED_CK_H;
  //OLED_CS=1;	//预制SLK和SS为高电平
  OLED_RST_L;
  delay_ms(50);
  OLED_RST_H;
  
  OLED_WrCmd(0xae);//--turn off oled panel
  OLED_WrCmd(0x00);//---set low column address
  OLED_WrCmd(0x10);//---set high column address
  OLED_WrCmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
  OLED_WrCmd(0x81);//--set contrast control register
  OLED_WrCmd(0xcf); // Set SEG Output Current Brightness
  OLED_WrCmd(0xa1);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
  OLED_WrCmd(0xc8);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
  OLED_WrCmd(0xa6);//--set normal display
  OLED_WrCmd(0xa8);//--set multiplex ratio(1 to 64)
  OLED_WrCmd(0x3f);//--1/64 duty
  OLED_WrCmd(0xd3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
  OLED_WrCmd(0x00);//-not offset
  OLED_WrCmd(0xd5);//--set display clock divide ratio/oscillator frequency
  OLED_WrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
  OLED_WrCmd(0xd9);//--set pre-charge period
  OLED_WrCmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  OLED_WrCmd(0xda);//--set com pins hardware configuration
  OLED_WrCmd(0x12);
  OLED_WrCmd(0xdb);//--set vcomh
  OLED_WrCmd(0x40);//Set VCOM Deselect Level
  OLED_WrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
  OLED_WrCmd(0x02);//
  OLED_WrCmd(0x8d);//--set Charge Pump enable/disable
  OLED_WrCmd(0x14);//--set(0x10) disable
  OLED_WrCmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
  OLED_WrCmd(0xa6);// Disable Inverse Display On (0xa6/a7)
  OLED_WrCmd(0xaf);//--turn on oled panel
  OLED_CLS();      //初始清屏
  OLED_Set_Pos(0,0);
}
//oled延时函数
void oled_short_delay(unsigned int Del)	//
{
  while(Del--)
  {
    __asm("NOP");
    
  }
}
//画点命令
void OLED_WrDat(unsigned char data)
{
  unsigned char i=8;
  //OLED_CS=0;;
  OLED_DC_H;;
  OLED_CK_L;;
  oled_short_delay(1);
  while(i--)
  {
    if(data&0x80){OLED_DI_H;}
    else{OLED_DI_L;}
    OLED_CK_H;
    oled_short_delay(1);
    //asm("nop");            
    OLED_CK_L;
    data<<=1;    
  }
  //OLED_CS=1;
}
//oled命令
void OLED_WrCmd(unsigned char cmd)
{
  unsigned char i=8;
  
  //OLED_CS=0;;
  OLED_DC_L;
  OLED_CK_L;
  oled_short_delay(1);
  while(i--)
  {
    if(cmd&0x80){OLED_DI_H;}
    else{OLED_DI_L;}
    OLED_CK_H;
    oled_short_delay(1);
    OLED_CK_L;
    cmd<<=1;
  } 
  //OLED_CS=1;
}
//oled设置点
void OLED_Set_Pos(unsigned char x, unsigned char y)
{ 
  OLED_WrCmd(0xb0+y);
  OLED_WrCmd(((x&0xf0)>>4)|0x10);
  OLED_WrCmd((x&0x0f));
} 
//oled全亮
void OLED_Fill(void)
{
  unsigned char y,x;
  
  for(y=0;y<8;y++)
  {
    OLED_WrCmd(0xb0+y);
    OLED_WrCmd(0x01);
    OLED_WrCmd(0x10);
    for(x=0;x<X_WIDTH;x++)
      OLED_WrDat(0xff);
  }
}
//oled清屏
void OLED_CLS(void)
{
  unsigned char y,x;	
  for(y=0;y<8;y++)
  {
    OLED_WrCmd(0xb0+y);
    OLED_WrCmd(0x01);
    OLED_WrCmd(0x10);
    for(x=0;x<X_WIDTH;x++)
      OLED_WrDat(0);
  }
}
//画点
//x0~127y0~63
//左上0，0右下127，63
void OLED_PutPixel(unsigned char x,unsigned char y)
{
  unsigned char data1;  //data1当前点的数据 
  
  OLED_Set_Pos(x,(unsigned char)(y>>3));
  data1 =(unsigned char)(0x01<<(y%8)); 	
  OLED_WrCmd((unsigned char)(0xb0+(y>>3)));
  OLED_WrCmd((unsigned char)(((x&0xf0)>>4)|0x10));
  OLED_WrCmd((unsigned char)((x&0x0f)|0x00));
  OLED_WrDat(data1);
}
//写字符串，清点就写”空格“
void OLED_P6x8Str(unsigned char x,unsigned char y, char ch[])
{
  unsigned char c=0,i=0,j=0;      
  while (ch[j]!='\0')
  {    
    c =ch[j]-32;
    if(x>126){x=0;y++;}
    OLED_Set_Pos(x,y);
    for(i=0;i<6;i++)     
      OLED_WrDat(F6x8[c][i]);
    x+=6;
    j++;
  }
}
//反色显示字符
void OLED_P6x8Rst(unsigned char x,unsigned char y, char ch[])
{
  
	uint8_t c=0,i=0,j=0;
	while (ch[j]!='\0')
	{
	  	c =ch[j]-32;
	  	if(x>126){x=0;y++;}
	  	OLED_Set_Pos(x,y);
	  	for(i=0;i<6;i++)	OLED_WrDat(~F6x8[c][i]);
	  	x+=6;
	  	j++;
	}
}
//写整数
void OLED_Print_Num(uint8_t x, uint8_t y, int num)
{
	char ch[7];
	sprintf(ch,"%d",num);
        OLED_P6x8Str(x, y,ch);
}
//写浮点数
void OLED_Print_Float(uint8_t x, uint8_t y, float num)
{
	char ch[10];
	snprintf(ch, 10, "%.4f", num);
	OLED_P6x8Str(x, y, ch);
}
//社团logo
void OLED_Logo(void)
{
    	uint8_t y,x;
    	for(y=1;y<8;y++)
	{
		OLED_WrCmd(0xb0+y);
		OLED_WrCmd(0x01);
		OLED_WrCmd(0x10);
		for(x=0;x<128;x++)	
                  OLED_WrDat(logobmp2[y][x]); 
	}
}
//写大字符串
void OLED_P8x16Str(unsigned char x,unsigned char y, char ch[])
{
  unsigned char c=0,i=0,j=0;
  
  while (ch[j]!='\0')
  {    
    c =ch[j]-32;
    if(x>120){x=0;y++;}
    OLED_Set_Pos(x,y);
    for(i=0;i<8;i++)     
      OLED_WrDat(F8X16[c*16+i]);
    OLED_Set_Pos(x,y+1);
    for(i=0;i<8;i++)     
      OLED_WrDat(F8X16[c*16+i+8]);
    x+=8;
    j++;
  }
}
////显示摄像头图片
//void OLED_Road(unsigned short high, unsigned short wide, unsigned char *Pixle)
//{ 	 
//  unsigned char i = 0, j = 0,temp=0;
//  unsigned char wide_start = (128 - wide)/2;
//  for(i=0;i<high-7;i+=8)
//  {
//    
//    OLED_Set_Pos(wide_start,i/8+1);
//    
//    for(j=0;j<wide;j++) 
//    { 
//      temp=0;
//      if(Pixle[(0+i)*wide + j]) temp|=1;
//      if(Pixle[(1+i)*wide + j]) temp|=2;
//      if(Pixle[(2+i)*wide + j]) temp|=4;
//      if(Pixle[(3+i)*wide + j]) temp|=8;
//      if(Pixle[(4+i)*wide + j]) temp|=0x10;
//      if(Pixle[(5+i)*wide + j]) temp|=0x20;
//      if(Pixle[(6+i)*wide + j]) temp|=0x40;
//      if(Pixle[(7+i)*wide + j]) temp|=0x80;
//      OLED_WrDat(temp);
//    }
//  }  
//}

