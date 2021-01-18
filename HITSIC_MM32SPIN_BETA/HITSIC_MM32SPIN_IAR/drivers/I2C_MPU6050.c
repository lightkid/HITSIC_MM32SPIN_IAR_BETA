
#include "I2C_MPU6050.h"
#include "string"
//mpu6050_data_t MPU6050[1];
//uint8_t data_MPU6050[14];

void inv_mpu6050_setDetfaultConfig(inv_mpu6050_t* s)
{
  memset(s, 0, sizeof(inv_mpu6050_t));
  //默认滤波
  inv_mpu6050_set_LPF(s, MPU_ABW_218, MPU_GBW_176);
  
  //默认量程
  inv_mpu6050_set_FS(s, MPU_FS_4G, MPU_FS_2000dps);
  
  //默认不改变坐标轴
  Axis_t acc, gyro;
  memset(&acc, 0, sizeof(Axis_t));
  memset(&gyro, 0, sizeof(Axis_t));
  acc.x2x = 1; gyro.x2x = 1;
  acc.y2y = 1; gyro.y2y = 1;
  acc.z2z = 1; gyro.z2z = 1;
  inv_mpu6050_set_Axis(s, &acc, &gyro);
}

void inv_mpu6050_set_serif(inv_mpu6050_t* s, inv_mpu6050_serif_t* serif_)
{
  memcpy(&(s->serif), serif_, sizeof(inv_mpu6050_serif_t));
}

void inv_mpu6050_set_LPF(inv_mpu6050_t* s, enum mpu_accel_bw accbw, enum  mpu_gyro_bw gyrobw)
{
  s->states.accel_bw = accbw;
  s->states.gyro_bw = gyrobw;
}

void inv_mpu6050_set_FS(inv_mpu6050_t* s, enum mpu_accel_fs accfs, enum  mpu_gyro_fs gyrofs)
{
  s->states.accel_fullscale = accfs;
  s->states.gyro_fullscale = gyrofs;
  switch (s->states.accel_fullscale)
  {
  case (uint8_t)MPU_FS_2G:
    s->states.acc_inv = 2.0 * 9.8 / 32768.0;
    break;
  case (uint8_t)MPU_FS_4G:
    s->states.acc_inv = 4.0 * 9.8 / 32768.0;
    break;
  case (uint8_t)MPU_FS_8G:
    s->states.acc_inv = 8.0 * 9.8 / 32768.0;
    break;
  case (uint8_t)MPU_FS_16G:
    s->states.acc_inv = 16.0 * 9.8 / 32768.0;
    break;
  default:
    s->states.acc_inv = 16.0 * 9.8 / 32768.0;
    break;
  }
  switch (s->states.gyro_fullscale)
  {
  case (uint8_t)MPU_FS_250dps:
    s->states.gyro_inv = /*0.0174532925199432957692369076848f * */(250.0 / 32768.0);
    break;
  case (uint8_t)MPU_FS_500dps:
    s->states.gyro_inv =/* 0.0174532925199432957692369076848f * */(500.0 / 32768.0);
    break;
  case (uint8_t)MPU_FS_1000dps:
    s->states.gyro_inv =/* 0.0174532925199432957692369076848f * */(1000.0 / 32768.0);
    break;
  case (uint8_t)MPU_FS_2000dps:
    s->states.gyro_inv =/* 0.0174532925199432957692369076848f * */(2000.0 / 32768.0);
    break;
  default:
    s->states.gyro_inv =/* 0.0174532925199432957692369076848f * */(2000.0 / 32768.0);
    break;
  }
}

int inv_mpu6050_init(inv_mpu6050_t* s, const char* nickname)
{
  uint8_t whoami = 0xff;
  //int times = 100;
  s->serif.i2c_slave_addr = IMU_MPU6050_I2C_ADDR;
  s->name = nickname;
  while (1)
  {
    inv_mpu6050_read_reg(s, IMU_MPU6050_WHO_AM_I, &whoami, 1);
    if (whoami == INV_MPU6050)
    {
      s->states.mpu = INV_MPU6050;
      //inv_mpu6050_message(inv_icm20602_information, "%s:select MPU6050", s->name);
      break;
    }
    else
    {
      //s->serif.i2c_slave_addr = s->serif.i2c_slave_addr ^ 0x01;//异或运算，翻转最低位，因为地址只有0x68和0x69
      //inv_mpu6050_delay_ms(10);
      //inv_icm20602_message(inv_icm20602_warning, "%s:select warning,%d times remaining", s->name, times);
      //inv_icm20602_message(inv_icm20602_warning, "%s:whoami = %#X", s->name, whoami);
      //--times;
      //if (times <= 0)
      //{
      // inv_icm20602_message(inv_icm20602_error, "%s:select error,%d times remaining", s->name, times);
      //  inv_icm20602_message(inv_icm20602_error, "%s:whoami = %#X", s->name, whoami);
      return -1;
      //}
    }
  }
  uint8_t val = 0x80;
  switch (s->states.mpu)//每种imu都一样的配置过程
  {
  default:
    val = 0x80;
    inv_mpu6050_write_reg(s, IMU_MPU6050_PWR_MGMT_1, &val, 1);//复位设备
    do
    {//等待复位成功
      inv_mpu6050_delay_ms(10);
      inv_mpu6050_read_reg(s, IMU_MPU6050_PWR_MGMT_1, &val, 1);
      //inv_icm20602_message(inv_icm20602_warning, "%s:wait...PWR_MGMT_1 = %#X", s->name, val);
    } while (!(0x41 == val || 0x01 == val || 0x40 == val));
    val = 0x00; inv_mpu6050_delay_ms(10); inv_mpu6050_write_reg(s, IMU_MPU6050_PWR_MGMT_1, &val, 1);                          //使用内部时钟
    val = 0x00; inv_mpu6050_delay_ms(10); inv_mpu6050_write_reg(s, IMU_MPU6050_PWR_MGMT_2, &val, 1);                          //开启陀螺仪和加速度计
    val = s->states.gyro_bw; inv_mpu6050_delay_ms(10); inv_mpu6050_write_reg(s, IMU_MPU6050_CONFIG, &val, 1);	             //配置陀螺仪数字滤波器
    val = s->states.sample_div; inv_mpu6050_delay_ms(10); inv_mpu6050_write_reg(s, IMU_MPU6050_SMPLRT_DIV, &val, 1);			  //采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    val = s->states.gyro_fullscale << 3; inv_mpu6050_delay_ms(10); inv_mpu6050_write_reg(s, IMU_MPU6050_GYRO_CONFIG, &val, 1);  //设置陀螺仪量程
    val = s->states.accel_fullscale << 3; inv_mpu6050_delay_ms(10); inv_mpu6050_write_reg(s, IMU_MPU6050_ACCEL_CONFIG, &val, 1);//设置加速度计量程
    val = s->states.accel_bw; inv_mpu6050_delay_ms(10); inv_mpu6050_write_reg(s, IMU_MPU6050_ACCEL_CONFIG_2, &val, 1);        //配置加速度计数字滤波器
  }
  inv_mpu6050_delay_ms(10);
  s->if_init = 1;
  return 0;
}

void inv_mpu6050_GYROoffset(inv_mpu6050_t* s)
{
  if (s->if_init == 0) {
    return;
  }
  uint32_t times = 1000;
  float buffgxo = 0;
  float buffgyo = 0;
  float buffgzo = 0;
  s->rawdata.gxo = 0;
  s->rawdata.gyo = 0;
  s->rawdata.gzo = 0;
  while (times--)
  {
    inv_mpu6050_poll_sensor_data_reg(s);
    inv_mpu6050_delay_ms(5);
    buffgxo += s->rawdata.gx;
    buffgyo += s->rawdata.gy;
    buffgzo += s->rawdata.gz;
  }
  s->rawdata.gxo = buffgxo * 0.001f;
  s->rawdata.gyo = buffgyo * 0.001f;
  s->rawdata.gzo = buffgzo * 0.001f;
}

void inv_mpu6050_ACCoffset(inv_mpu6050_t* s, float axo_, float ayo_, float azo_)
{
  s->rawdata.axo = axo_;
  s->rawdata.ayo = ayo_;
  s->rawdata.azo = azo_;
}

void inv_mpu6050_read_reg(inv_mpu6050_t* s, uint8_t reg, uint8_t* data, uint32_t len)
{
  SOFT_I2C_Read(s->serif.i2c_slave_addr, reg, data, len);
  //  int result = 1;
  //  if (s->serif.is_spi == 1)
  //  {
  //    if (len > sizeof(s->serif.spi_transferTxBuf) - 1)
  //    {
  //      return;
  //    }
  //    memset(s->serif.spi_transferTxBuf, 0, len + 1);
  //    s->serif.spi_transferTxBuf[0] = reg | 0x80;
  //    result = s->serif.spi_transfer(s->serif.spi_base, s->serif.spi_pcs, s->serif.spi_transferRxBuf, s->serif.spi_transferTxBuf, len + 1);
  //    memcpy(data, s->serif.spi_transferRxBuf + 1, len);
  //  }
  //  else
  //  {
  //result = s->serif.i2c_read_reg(s->serif.i2c_base, s->serif.i2c_slave_addr, reg, data, len);
  //  }
  //  if (result != 0)
  //  {
  //    inv_icm20602_message(inv_icm20602_warning, "%s:Icm Read error:code=%d", s->name, result);
  //  }
}

void inv_mpu6050_write_reg(inv_mpu6050_t* s, uint8_t reg, uint8_t* data, uint32_t len)
{
  SOFT_I2C_Write(s->serif.i2c_slave_addr, reg, data, len);
  //  int result = 1;
  //  if (s->serif.is_spi == 1)
  //  {
  //    if (len > sizeof(s->serif.spi_transferTxBuf) - 1)
  //    {
  //      return;
  //    }
  //    s->serif.spi_transferTxBuf[0] = reg;//只有读的时候要并上高位
  //    memcpy(s->serif.spi_transferTxBuf + 1, data, len);
  //    result = s->serif.spi_transfer(s->serif.spi_base, s->serif.spi_pcs, s->serif.spi_transferRxBuf, s->serif.spi_transferTxBuf, len + 1);
  //  }
  //  else
  //  {
  //result = s->serif.i2c_write_reg(s->serif.i2c_base, s->serif.i2c_slave_addr, reg, data, len);
  //  }
  //  if (result != 0)
  //  {
  //    inv_icm20602_message(inv_icm20602_warning, "%s:Icm Write error:code=%d", s->name, result);
  //  }
}

void inv_mpu6050_poll_sensor_data_reg(inv_mpu6050_t* s)
{
  inv_mpu6050_read_reg(s, IMU_MPU6050_ACCEL_XOUT_H, s->rawdata.buff, 14);
  s->rawdata.ax = s->states.acc_inv * ((int16_t)(s->rawdata.buff[0] << 8) | s->rawdata.buff[1]) - s->rawdata.axo;
  s->rawdata.ay = s->states.acc_inv * ((int16_t)(s->rawdata.buff[2] << 8) | s->rawdata.buff[3]) - s->rawdata.ayo;
  s->rawdata.az = s->states.acc_inv * ((int16_t)(s->rawdata.buff[4] << 8) | s->rawdata.buff[5]) - s->rawdata.azo;
  s->temp = 0.02f * (25.0f + 0.003059975520f * ((int16_t)(s->rawdata.buff[6] << 8) | s->rawdata.buff[7])) + 0.98f * s->temp;
  s->rawdata.gx = s->states.gyro_inv * ((int16_t)(s->rawdata.buff[8] << 8) | s->rawdata.buff[9]) - s->rawdata.gxo;
  s->rawdata.gy = s->states.gyro_inv * ((int16_t)(s->rawdata.buff[10] << 8) | s->rawdata.buff[11]) - s->rawdata.gyo;
  s->rawdata.gz = s->states.gyro_inv * ((int16_t)(s->rawdata.buff[12] << 8) | s->rawdata.buff[13]) - s->rawdata.gzo;
  s->rawdata.neg_ax = -s->rawdata.ax;
  s->rawdata.neg_ay = -s->rawdata.ay;
  s->rawdata.neg_az = -s->rawdata.az;
  s->rawdata.neg_gx = -s->rawdata.gx;
  s->rawdata.neg_gy = -s->rawdata.gy;
  s->rawdata.neg_gz = -s->rawdata.gz;
  s->acc.x = *s->p_acc.x;
  s->acc.y = *s->p_acc.y;
  s->acc.z = *s->p_acc.z;
  s->gyro.x = *s->p_gyro.x;
  s->gyro.y = *s->p_gyro.y;
  s->gyro.z = *s->p_gyro.z;
}

void inv_mpu6050_set_Axis(inv_mpu6050_t* s, Axis_t* fa, Axis_t* fg)
{
  //acc_x
  if (fa->x2x)
  {
    if (fa->x2x > 0)
    {
      s->p_acc.x = &s->rawdata.ax;
    }
    else
    {
      s->p_acc.x = &s->rawdata.neg_ax;
    }
  }
  else if (fa->x2y)
  {
    if (fa->x2y > 0)
    {
      s->p_acc.x = &s->rawdata.ay;
    }
    else
    {
      s->p_acc.x = &s->rawdata.neg_ay;
    }
  }
  else if (fa->x2z)
  {
    if (fa->x2z > 0)
    {
      s->p_acc.x = &s->rawdata.az;
    }
    else
    {
      s->p_acc.x = &s->rawdata.neg_az;
    }
  }
  
  //acc_y
  if (fa->y2x)
  {
    if (fa->y2x > 0)
    {
      s->p_acc.y = &s->rawdata.ax;
    }
    else
    {
      s->p_acc.y = &s->rawdata.neg_ax;
    }
  }
  else if (fa->y2y)
  {
    if (fa->y2y > 0)
    {
      s->p_acc.y = &s->rawdata.ay;
    }
    else
    {
      s->p_acc.y = &s->rawdata.neg_ay;
    }
  }
  else if (fa->y2z)
  {
    if (fa->y2z > 0)
    {
      s->p_acc.y = &s->rawdata.az;
    }
    else
    {
      s->p_acc.y = &s->rawdata.neg_az;
    }
  }
  
  //acc_z
  if (fa->z2x)
  {
    if (fa->z2x > 0)
    {
      s->p_acc.z = &s->rawdata.ax;
    }
    else
    {
      s->p_acc.z = &s->rawdata.neg_ax;
    }
  }
  else if (fa->z2y)
  {
    if (fa->z2y > 0)
    {
      s->p_acc.z = &s->rawdata.ay;
    }
    else
    {
      s->p_acc.z = &s->rawdata.neg_ay;
    }
  }
  else if (fa->z2z)
  {
    if (fa->z2z > 0)
    {
      s->p_acc.z = &s->rawdata.az;
    }
    else
    {
      s->p_acc.z = &s->rawdata.neg_az;
    }
  }
  
  //以下都是gyro部分
  //gyro_x
  if (fg->x2x)
  {
    if (fg->x2x > 0)
    {
      s->p_gyro.x = &s->rawdata.gx;
    }
    else
    {
      s->p_gyro.x = &s->rawdata.neg_gx;
    }
  }
  else if (fg->x2y)
  {
    if (fg->x2y > 0)
    {
      s->p_gyro.x = &s->rawdata.gy;
    }
    else
    {
      s->p_gyro.x = &s->rawdata.neg_gy;
    }
  }
  else if (fg->x2z)
  {
    if (fg->x2z > 0)
    {
      s->p_gyro.x = &s->rawdata.gz;
    }
    else
    {
      s->p_gyro.x = &s->rawdata.neg_gz;
    }
  }
  
  //gyro_y
  if (fg->y2x)
  {
    if (fg->y2x > 0)
    {
      s->p_gyro.y = &s->rawdata.gx;
    }
    else
    {
      s->p_gyro.y = &s->rawdata.neg_gx;
    }
  }
  else if (fg->y2y)
  {
    if (fg->y2y > 0)
    {
      s->p_gyro.y = &s->rawdata.gy;
    }
    else
    {
      s->p_gyro.y = &s->rawdata.neg_gy;
    }
  }
  else if (fg->y2z)
  {
    if (fg->y2z > 0)
    {
      s->p_gyro.y = &s->rawdata.gz;
    }
    else
    {
      s->p_gyro.y = &s->rawdata.neg_gz;
    }
  }
  
  //gyro_z
  if (fg->z2x)
  {
    if (fg->z2x > 0)
    {
      s->p_gyro.z = &s->rawdata.gx;
    }
    else
    {
      s->p_gyro.z = &s->rawdata.neg_gx;
    }
  }
  else if (fg->z2y)
  {
    if (fg->z2y > 0)
    {
      s->p_gyro.z = &s->rawdata.gy;
    }
    else
    {
      s->p_gyro.z = &s->rawdata.neg_gy;
    }
  }
  else if (fg->z2z)
  {
    if (fg->z2z > 0)
    {
      s->p_gyro.z = &s->rawdata.gz;
    }
    else
    {
      s->p_gyro.z = &s->rawdata.neg_gz;
    }
  }
}

void inv_mpu6050_delay_ms(uint32_t ms)
{
  while (--ms)
  {
    uint32_t x = 1500;//???
    while (--x)
    {
      
    }
  }
}

inv_mpu6050_t icm1;


void MPU6050_Init(void)
{
  inv_mpu6050_setDetfaultConfig(&icm1);//必须调用这个函数进行默认配置
  SOFT_I2C_Init();//I2C初始化
  inv_mpu6050_init(&icm1, "icm01");
}
