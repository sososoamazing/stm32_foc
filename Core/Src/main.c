/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <math.h>
#include "AS5600.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

float voltage_power_supply=12.6;
float Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0;
#define _3PI_2 4.71238898038f
float zero_electric_angle=0;
int PP=1,DIR=1;



int Sensor_DIR=-1;    //����������
int Motor_PP=7;    //���������
float motor_target=1;

//��ʼ��������������
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//�궨��ʵ�ֵ�һ��Լ������,��������һ��ֵ�ķ�Χ��
//������˵���ú궨�������Ϊ _constrain�������������� amt��low �� high���ֱ��ʾҪ���Ƶ�ֵ����Сֵ�����ֵ���ú궨���ʵ��ʹ������Ԫ����������� amt �Ƿ�С�� low ����� high���������е�������Сֵ�����߷���ԭֵ��
//���仰˵����� amt С�� low���򷵻� low����� amt ���� high���򷵻� high�����򷵻� amt��������_constrain(amt, low, high) �ͻὫ amt Լ���� [low, high] �ķ�Χ�ڡ�1


// ��һ���Ƕȵ� [0,2PI]
float _normalizeAngle(float angle){
  float a = fmod(angle, 2*PI);   //ȡ������������ڹ�һ�����г�����ֵ�������֪
  return a >= 0 ? a : (a + 2*PI);  
  //��Ŀ���������ʽ��condition ? expr1 : expr2 
  //���У�condition ��Ҫ��ֵ���������ʽ����������������򷵻� expr1 ��ֵ�����򷵻� expr2 ��ֵ�����Խ���Ŀ�������Ϊ if-else ���ļ���ʽ��
  //fmod �����������ķ����������ͬ����ˣ��� angle ��ֵΪ����ʱ�������ķ��Ž��� _2PI �ķ����෴��Ҳ����˵����� angle ��ֵС�� 0 �� _2PI ��ֵΪ�������� fmod(angle, _2PI) ��������Ϊ������
  //���磬�� angle ��ֵΪ -PI/2��_2PI ��ֵΪ 2PI ʱ��fmod(angle, _2PI) ������һ������������������£�����ͨ������������������ _2PI �����Ƕȹ�һ���� [0, 2PI] �ķ�Χ�ڣ���ȷ���Ƕȵ�ֵʼ��Ϊ������
}


void setPwm(float Ua, float Ub, float Uc) {
  // ����ռ�ձ�
  // ����ռ�ձȴ�0��1
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  // ��ռ�ձ�ӳ�䵽��ʱ����ARR��Χ��
  uint16_t pwm_a = (uint16_t)(dc_a * (__HAL_TIM_GET_AUTORELOAD(&htim1)));
  uint16_t pwm_b = (uint16_t)(dc_b * (__HAL_TIM_GET_AUTORELOAD(&htim1)));
  uint16_t pwm_c = (uint16_t)(dc_c * (__HAL_TIM_GET_AUTORELOAD(&htim1)));

  // ����PWM�ź�
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_a);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_b);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_c);
}


void setTorque(float Uq,float angle_el) {
  Uq=_constrain(Uq,-voltage_power_supply/2,voltage_power_supply/2);
  float Ud=0;
  angle_el = _normalizeAngle(angle_el);
  // ������任
  Ualpha =  -Uq*sin(angle_el); 
  Ubeta =   Uq*cos(angle_el); 

  // ��������任
  Ua = Ualpha + voltage_power_supply/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
  setPwm(Ua,Ub,Uc);
}
//�������ĵ�Ƕȣ������˷��򡢼������Լ����ܵ�ƫ��У��������������ع淶����ĵ�Ƕ�ֵ��
float _electricalAngle(){
  return  _normalizeAngle((float)(DIR *  PP) * getAngle_Without_track()-zero_electric_angle);
}
//�Դ���������У׼����ȷ����������Ƕ�
void DFOC_alignSensor(int _PP,int _DIR)
{ 
  PP=_PP;
  DIR=_DIR;
  setTorque(3, _3PI_2);
  HAL_Delay(3000);
  zero_electric_angle=_electricalAngle();
  setTorque(0, _3PI_2);

}
//��ȡ��ǰ����ĽǶ�
float DFOC_M0_Angle()
{
  return getAngle();
}



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	
  DFOC_alignSensor(Motor_PP,Sensor_DIR);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		 //����Ƕ�ֵ
  //Serial.print("��ǰ�Ƕȣ�");
  //Serial.println(DFOC_M0_Angle());
  //����Ƕ�ֵ
  float Kp=0.133;
  float Sensor_Angle=DFOC_M0_Angle();
  setTorque(motor_target,_electricalAngle());   //��ѹ����
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
