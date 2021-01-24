#ifndef __I2C_MPU6050_H_
#define __I2C_MPU6050_H_

#include "board.h"

#define	    IMU_MPU6050_I2C_ADDR	   0xD0 

#define     IMU_MPU6050_XG_OFFS_TC_H       0x04
#define     IMU_MPU6050_XG_OFFS_TC_L       0x05
#define     IMU_MPU6050_YG_OFFS_TC_H       0x07
#define     IMU_MPU6050_YG_OFFS_TC_L       0x08
#define     IMU_MPU6050_ZG_OFFS_TC_H       0x0A
#define     IMU_MPU6050_ZG_OFFS_TC_L       0x0B
#define     IMU_MPU6050_SELF_TEST_X_ACCEL  0x0D
#define     IMU_MPU6050_SELF_TEST_Y_ACCEL  0x0E
#define     IMU_MPU6050_SELF_TEST_Z_ACCEL  0x0F
#define     IMU_MPU6050_XG_OFFS_USRH       0x13
#define     IMU_MPU6050_XG_OFFS_USRL       0x14
#define     IMU_MPU6050_YG_OFFS_USRH       0x15
#define     IMU_MPU6050_YG_OFFS_USRL       0x16
#define     IMU_MPU6050_ZG_OFFS_USRH       0x17
#define     IMU_MPU6050_ZG_OFFS_USRL       0x18
#define     IMU_MPU6050_SMPLRT_DIV         0x19
#define     IMU_MPU6050_CONFIG             0x1A
#define     IMU_MPU6050_GYRO_CONFIG        0x1B
#define     IMU_MPU6050_ACCEL_CONFIG       0x1C
#define     IMU_MPU6050_ACCEL_CONFIG_2     0x1D
#define     IMU_MPU6050_LP_MODE_CFG        0x1E
#define     IMU_MPU6050_ACCEL_WOM_X_THR    0x20
#define     IMU_MPU6050_ACCEL_WOM_Y_THR    0x21
#define     IMU_MPU6050_ACCEL_WOM_Z_THR    0x22
#define     IMU_MPU6050_FIFO_EN            0x23
#define     IMU_MPU6050_FSYNC_INT          0x36
#define     IMU_MPU6050_INT_PIN_CFG        0x37
#define     IMU_MPU6050_INT_ENABLE         0x38
#define     IMU_MPU6050_FIFO_WM_INT_STATUS 0x39 
#define     IMU_MPU6050_INT_STATUS         0x3A
#define     IMU_MPU6050_ACCEL_XOUT_H       0x3B
#define     IMU_MPU6050_ACCEL_XOUT_L       0x3C
#define     IMU_MPU6050_ACCEL_YOUT_H       0x3D
#define     IMU_MPU6050_ACCEL_YOUT_L       0x3E
#define     IMU_MPU6050_ACCEL_ZOUT_H       0x3F
#define     IMU_MPU6050_ACCEL_ZOUT_L       0x40
#define     IMU_MPU6050_TEMP_OUT_H         0x41
#define     IMU_MPU6050_TEMP_OUT_L         0x42
#define     IMU_MPU6050_GYRO_XOUT_H        0x43
#define     IMU_MPU6050_GYRO_XOUT_L        0x44
#define     IMU_MPU6050_GYRO_YOUT_H        0x45
#define     IMU_MPU6050_GYRO_YOUT_L        0x46
#define     IMU_MPU6050_GYRO_ZOUT_H        0x47
#define     IMU_MPU6050_GYRO_ZOUT_L        0x48
#define     IMU_MPU6050_SELF_TEST_X_GYRO   0x50
#define     IMU_MPU6050_SELF_TEST_Y_GYRO   0x51
#define     IMU_MPU6050_SELF_TEST_Z_GYRO   0x52
#define     IMU_MPU6050_FIFO_WM_TH1        0x60
#define     IMU_MPU6050_FIFO_WM_TH2        0x61
#define     IMU_MPU6050_SIGNAL_PATH_RESET  0x68
#define     IMU_MPU6050_ACCEL_INTEL_CTRL   0x69
#define     IMU_MPU6050_USER_CTRL          0x6A
#define     IMU_MPU6050_PWR_MGMT_1         0x6B
#define     IMU_MPU6050_PWR_MGMT_2         0x6C
#define     IMU_MPU6050_I2C_IF             0x70
#define     IMU_MPU6050_FIFO_COUNTH        0x72
#define     IMU_MPU6050_FIFO_COUNTL        0x73
#define     IMU_MPU6050_FIFO_R_W           0x74
#define     IMU_MPU6050_WHO_AM_I           0x75
#define     IMU_MPU6050_XA_OFFSET_H        0x77
#define     IMU_MPU6050_XA_OFFSET_L        0x78
#define     IMU_MPU6050_YA_OFFSET_H        0x7A
#define     IMU_MPU6050_YA_OFFSET_L        0x7B
#define     IMU_MPU6050_ZA_OFFSET_H        0x7D
#define     IMU_MPU6050_ZA_OFFSET_L        0x7E
#define     INV_MPU6050                    0x68
enum mpu_accel_fs {	// In the ACCEL_CONFIG (0x1C) register,[4,3] the full scale select  bits are :
    MPU_FS_2G = 0,	// 00 = 2G
    MPU_FS_4G,		// 01 = 4
	MPU_FS_8G,		// 10 = 8
    MPU_FS_16G,		// 11 = 16
    NUM_MPU_AFS
};

enum mpu_accel_ois_fs {	// In the ACCEL_CONFIG (0x1C) register, the accel ois full scale select bits are :
    MPU_OIS_FS_2G = 0,	// 00 = 2G
    MPU_OIS_FS_4G,		// 01 = 4
	MPU_OIS_FS_8G,		// 10 = 8
    MPU_OIS_FS_1G,		// 11 = 1
    NUM_MPU_OIS_AFS
};

enum mpu_accel_bw {		// In the ACCEL_CONFIG2 (0x1D) register, [2,0]the BW setting bits are :
	MPU_ABW_218 = 1,	// 001 = 218 Hz
	MPU_ABW_99,			// 010 = 99 Hz
	MPU_ABW_45,			// 011 = 45 Hz
	MPU_ABW_21,			// 100 = 21 Hz
	MPU_ABW_10,			// 101 = 10 Hz
	MPU_ABW_5,			// 110 = 5 Hz
	MPU_ABW_420,		// 111 = 420 Hz
	NUM_MPU_ABW
};

enum mpu_gyro_fs {		// In the GYRO_CONFIG register, the [4,3]fS_SEL bits are :
	MPU_FS_250dps = 0,	// 00 = 250
	MPU_FS_500dps,		// 01 = 500
	MPU_FS_1000dps,		// 10 = 1000
	MPU_FS_2000dps,		// 11 = 2000
	NUM_MPU_GFS
};

enum mpu_gyro_bw {   // In the CONFIG register, the  [2,0]BW setting bits are :
	MPU_GBW_250 = 0,  //000 = 250hz
	MPU_GBW_176 = 1, // 001 = 176 Hz
	MPU_GBW_92,		 // 010 = 92 Hz
	MPU_GBW_41,		 // 011 = 41 Hz
	MPU_GBW_20,		 // 100 = 20 Hz
	MPU_GBW_10,		 // 101 = 10 Hz
	MPU_GBW_5,		 // 110 = 5 Hz
    NUM_MPU_GBW
};

#ifndef INV_MIN
#define INV_MIN(x,y) (((x)<(y))?(x):(y))
#endif

#ifndef INV_MAX
#define INV_MAX(x,y) (((x)>(y))?(x):(y))
#endif

#ifndef INV_ABS
#define INV_ABS(x) (((x) < 0) ? -(x) : (x))
#endif

#define DEG2RAD (0.0174532925199432957692369076848f)
#define RAD2DEG (57.295779513082320876798154814105f)

typedef struct inv_mpu6050_serif
{
  uint8_t i2c_slave_addr;
}inv_mpu6050_serif_t;

typedef struct AxisTransformation { //转换表的意思是 例如t2g,意味着新的t轴对应这原来坐标系的g轴，最终的是新坐标系，新在前，旧在后
  int8_t x2x;
  int8_t x2y;
  int8_t x2z;
  int8_t y2x;
  int8_t y2y;
  int8_t y2z;
  int8_t z2x;
  int8_t z2y;
  int8_t z2z;
} Axis_t;

typedef struct inv_mpu6050//对象
{
  inv_mpu6050_serif_t serif;
  struct inv_mpu6050_states//配置
  {
    uint8_t mpu;
    uint8_t gyro_fullscale;
    uint8_t accel_fullscale;
    float gyro_inv;
    float acc_inv;
    uint8_t gyro_bw;
    uint8_t accel_bw;
    uint8_t sample_div;
  }states;
  struct inv_mpu6050_data {//传感器数据acc, gyro,由驱动更新；acc_dot, gyro_dot供用户使用
    volatile float x;
    volatile float y;
    volatile float z;
  } acc, gyro, acc_dot, gyro_dot;
  volatile float temp;//传感器数据temp，温度
  struct inv_mpu6050_pointer_2_data {//实现转换坐标轴的指针
    float* x;
    float* y;
    float* z;
  } p_acc, p_gyro;
  struct inv_mpu6050_rawdata {//原始数据
    uint8_t buff[14];
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float neg_ax;
    float neg_ay;
    float neg_az;
    float neg_gx;
    float neg_gy;
    float neg_gz;
    float axo;
    float ayo;
    float azo;
    float gxo;
    float gyo;
    float gzo;
  } rawdata;
  const char* name;
  uint8_t if_init;//初始化成功为1
}inv_mpu6050_t;

extern inv_mpu6050_t icm1;

void inv_mpu6050_setDetfaultConfig(inv_mpu6050_t* s);
void inv_mpu6050_set_serif(inv_mpu6050_t* s, inv_mpu6050_serif_t* serif_);
void inv_mpu6050_set_LPF(inv_mpu6050_t* s, enum mpu_accel_bw accbw, enum  mpu_gyro_bw gyrobw);
void inv_mpu6050_set_FS(inv_mpu6050_t* s, enum mpu_accel_fs accfs, enum  mpu_gyro_fs gyrofs);
int inv_mpu6050_init(inv_mpu6050_t* s, const char* nickname);
void inv_mpu6050_GYROoffset(inv_mpu6050_t* s);
void inv_mpu6050_ACCoffset(inv_mpu6050_t* s, float axo_, float ayo_, float azo_);
void inv_mpu6050_read_reg(inv_mpu6050_t* s, uint8_t reg, uint8_t* data, uint32_t len);
void inv_mpu6050_write_reg(inv_mpu6050_t* s, uint8_t reg, uint8_t* data, uint32_t len);
void inv_mpu6050_poll_sensor_data_reg(inv_mpu6050_t* s);
void inv_mpu6050_set_Axis(inv_mpu6050_t* s, Axis_t* fa, Axis_t* fg);
void inv_mpu6050_delay_ms(uint32_t ms);
void MPU6050_Init(void);

#endif