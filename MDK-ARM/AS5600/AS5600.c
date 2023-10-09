#include "AS5600.h"
#include <stdint.h>
#include "i2c.h"
#include <stdlib.h>
#include <math.h>
#include "main.h"



#define HAL_DELAY 100


int _raw_ang_hi = 0x0c;
int _raw_ang_lo = 0x0d;
int _ams5600_Address = 0x36;
int ledtime = 0;
int32_t full_rotations=0; // full rotation tracking;
float angle_prev=0; 

/*
DevAddress：设备的I2C地址，以16位无符号整数表示。
RegAddr：要读取的寄存器地址，以8位无符号整数表示。
pData：用于存储读取数据的指针。这是一个指向 uint8_t 类型的指针，表示一个字节数组。
Size：要读取的数据字节数
*/
void I2C_ReadTwoBytes(uint16_t DevAddress, uint8_t RegAddr, uint8_t* pData, uint16_t Size) {
  if (HAL_I2C_Mem_Read(&hi2c1, DevAddress, RegAddr, I2C_MEMADD_SIZE_8BIT, pData, Size, HAL_DELAY) != HAL_OK) {
    Error_Handler();
  }
}

/*
in_adr_hi和in_adr_lo，它们用于指定需要读取的两个字节数据的地址
*/
uint16_t readTwoBytes(uint8_t in_adr_hi, uint8_t in_adr_lo) {
  uint8_t buffer[2];
  uint16_t retVal;

  // 读取低位
  I2C_ReadTwoBytes(_ams5600_Address, in_adr_lo, buffer, 1);

  // 读取高位
  I2C_ReadTwoBytes(_ams5600_Address, in_adr_hi, buffer + 1, 1);

  // 合并成一个16位整数
  retVal = (buffer[1] << 8) | buffer[0];

  return retVal;
}

float getRawAngle()
{
  return readTwoBytes(_raw_ang_hi, _raw_ang_lo);
}

float getAngle_Without_track()
{
  return getRawAngle()*0.08789* PI / 180;    //得到弧度制的角度
}

float getAngle()
{
    float val = getAngle_Without_track();
    float d_angle = val - angle_prev;
    //计算旋转的总圈数
    //通过判断角度变化是否大于80%的一圈(0.8f*6.28318530718f)来判断是否发生了溢出，如果发生了，则将full_rotations增加1（如果d_angle小于0）或减少1（如果d_angle大于0）。
    if(fabs(d_angle) > (0.8f*6.28318530718f) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
    return (float)full_rotations * 6.28318530718f + angle_prev;
    
}
