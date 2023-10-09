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
DevAddress���豸��I2C��ַ����16λ�޷���������ʾ��
RegAddr��Ҫ��ȡ�ļĴ�����ַ����8λ�޷���������ʾ��
pData�����ڴ洢��ȡ���ݵ�ָ�롣����һ��ָ�� uint8_t ���͵�ָ�룬��ʾһ���ֽ����顣
Size��Ҫ��ȡ�������ֽ���
*/
void I2C_ReadTwoBytes(uint16_t DevAddress, uint8_t RegAddr, uint8_t* pData, uint16_t Size) {
  if (HAL_I2C_Mem_Read(&hi2c1, DevAddress, RegAddr, I2C_MEMADD_SIZE_8BIT, pData, Size, HAL_DELAY) != HAL_OK) {
    Error_Handler();
  }
}

/*
in_adr_hi��in_adr_lo����������ָ����Ҫ��ȡ�������ֽ����ݵĵ�ַ
*/
uint16_t readTwoBytes(uint8_t in_adr_hi, uint8_t in_adr_lo) {
  uint8_t buffer[2];
  uint16_t retVal;

  // ��ȡ��λ
  I2C_ReadTwoBytes(_ams5600_Address, in_adr_lo, buffer, 1);

  // ��ȡ��λ
  I2C_ReadTwoBytes(_ams5600_Address, in_adr_hi, buffer + 1, 1);

  // �ϲ���һ��16λ����
  retVal = (buffer[1] << 8) | buffer[0];

  return retVal;
}

float getRawAngle()
{
  return readTwoBytes(_raw_ang_hi, _raw_ang_lo);
}

float getAngle_Without_track()
{
  return getRawAngle()*0.08789* PI / 180;    //�õ������ƵĽǶ�
}

float getAngle()
{
    float val = getAngle_Without_track();
    float d_angle = val - angle_prev;
    //������ת����Ȧ��
    //ͨ���жϽǶȱ仯�Ƿ����80%��һȦ(0.8f*6.28318530718f)���ж��Ƿ������������������ˣ���full_rotations����1�����d_angleС��0�������1�����d_angle����0����
    if(fabs(d_angle) > (0.8f*6.28318530718f) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
    return (float)full_rotations * 6.28318530718f + angle_prev;
    
}
