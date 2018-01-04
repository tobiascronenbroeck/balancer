/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "can.h"
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t VoltagesRAW[9];
uint16_t CurrentsRAW[7];
uint32_t VoltagesRAWAvg[9];

double Voltages[7];
double Currents[7];
double sysTemp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


void balancerwritetouart1(char text[])
{
	HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen((const char*)text), HAL_MAX_DELAY);
}

void  startDACoutput() //Enabling Output of Reference Voltage for DC/DC Converter controll
{
	HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
}

void  startADCsampling()
{
	balancerwritetouart1("Starting ADC Sampling...\n");
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)VoltagesRAW, 9);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)CurrentsRAW, 7);
	balancerwritetouart1("Success!\n");
}

void calcTemp()
{
	double sysTempTemp= VoltagesRAW[7];
	sysTempTemp*=3300;
	sysTempTemp/=0xfff;
	sysTempTemp/=1000.0;
	sysTempTemp-=0.760;
	sysTempTemp/=0.0025;
	sysTempTemp+=25.0;
	sysTemp=sysTempTemp;
}

double calcinputvoltage()

{
	double temp= VoltagesRAWAvg[6];

	double output = 0;
	output= 0.0089*temp-0.3878; //0,3727

	return output;
}

void convertRAWtoVoltage()
{
	Voltages[0]=0.0974+((double)VoltagesRAWAvg[0]*0.0022);
	Voltages[1]=(-1.2495)+((double)VoltagesRAWAvg[1]*0.0037);
	Voltages[2]=(-0.2700)+((double)VoltagesRAWAvg[2]*0.0019);
	Voltages[3]=0.0133+((double)VoltagesRAWAvg[3]*0.0016);
	Voltages[4]=0.1447+((double)VoltagesRAWAvg[4]*0.0016);
	Voltages[5]=0.6459+((double)VoltagesRAWAvg[5]*0.0014);
	Voltages[6]=calcinputvoltage();
}

void convertRAWtoCurrent()
{
	double multiplicator = 0.0008056640625;
	Currents[0]=CurrentsRAW[0]*multiplicator;
	Currents[1]=CurrentsRAW[1]*multiplicator;
	Currents[2]=CurrentsRAW[2]*multiplicator;
	Currents[3]=CurrentsRAW[3]*multiplicator;
	Currents[4]=CurrentsRAW[4]*multiplicator;
	Currents[5]=CurrentsRAW[5]*multiplicator;
	Currents[6]=CurrentsRAW[6]*multiplicator;
}

void printRAWVoltages2UART1()
{
	char transmit[60];
	sprintf(transmit, "V_ADC: %d %d %d %d %d %d %d Temp: %d\n", VoltagesRAW[0], VoltagesRAW[1], VoltagesRAW[2], VoltagesRAW[3], VoltagesRAW[4], VoltagesRAW[5], VoltagesRAW[6],(int)sysTemp);
	balancerwritetouart1(transmit);
}

void printVoltages2UART1()
{
	char transmit[80];
	sprintf(transmit, "UC1 %2.2lfV UC2 %2.2lfV UC3 %2.2lfV UC4 %2.2lfV UC5 %2.2lfV UC6 %2.2lfV VBat %2.2lfV Temp: %d\n", Voltages[0], Voltages[1], Voltages[2], Voltages[3], Voltages[4], Voltages[5], Voltages[6],(int)sysTemp);
	balancerwritetouart1(transmit);
}

void printabsRAWVoltages2UART1()
{
	char transmit[60];
	sprintf(transmit, "V_ADC: %d %d %d %d %d %d %d Temp: %d\n", VoltagesRAWAvg[0], VoltagesRAWAvg[1], VoltagesRAWAvg[2], VoltagesRAWAvg[3], VoltagesRAWAvg[4], VoltagesRAWAvg[5], VoltagesRAWAvg[6],(int)sysTemp);
	balancerwritetouart1(transmit);
}

void printRAWCurrents2UART1()
{

	char transmit[40];
	sprintf(transmit, "I_ADC: %d %d %d %d %d %d %d \n", CurrentsRAW[0], CurrentsRAW[1], CurrentsRAW[2], CurrentsRAW[3], CurrentsRAW[4], CurrentsRAW[5], CurrentsRAW[6]);
	balancerwritetouart1(transmit);
}

void calculateaveragevoltage()
{
	uint32_t VoltagesRAWAvgtemp[9]={0};
	int abssamples=600;
	for(int samples=0; samples < abssamples; samples++){
	for(int ch=0; ch<9; ch++)
	{
		VoltagesRAWAvgtemp[ch]=VoltagesRAWAvgtemp[ch]+VoltagesRAW[ch];
	}
	HAL_Delay(10);
	}
	for(int i=0; i<9;i++)
	{
		VoltagesRAWAvg[i]=VoltagesRAWAvgtemp[i]/abssamples;
	}
}

void startTimers()
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
}

uint16_t getaverageofadcvalue()
{
	uint32_t sampletime= 150;
	uint32_t temp=0;
	for(int counter=0; counter <= sampletime; counter++)
	{
		temp += VoltagesRAW[6];
		HAL_Delay(100);
	}
	temp=temp/sampletime;
	return (uint16_t)temp;
}

void sendcalibdatatoserial()
{
	uint16_t avg= getaverageofadcvalue();
	char transmit[25];
	sprintf(transmit, "Avg. ADC Value: %d \n",avg);
	balancerwritetouart1(transmit);

}




// 2nd HAL Layer for Charger Circuit
void setCell1PWM(int value){__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,value);}
void setCell2PWM(int value){__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,value);}
void setCell3PWM(int value){__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,value);}
void setCell4PWM(int value){__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,value);}
void setCell5PWM(int value){__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,value);}
void setCell6PWM(int value){__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,value);}
void setFANPWM(int value){__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,value);}
void connectBatteryGND(){	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10,GPIO_PIN_SET);}
void disconnectBatteryGND(){	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10,GPIO_PIN_RESET);}
void disableAllBalancer(){setCell1PWM(0); setCell2PWM(0); setCell3PWM(0); setCell4PWM(0); setCell5PWM(0); setCell6PWM(0); setFANPWM(0);}
void setDACLevel(uint32_t output){HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,output);}

void calibrateVoltageInput()
{
	for (int i= 0; i<=3300; i=i+100)
	{
		setDACLevel(i);
		//HAL_Delay(2000);
		char transmit[40];
		calculateaveragevoltage();
		convertRAWtoVoltage();
		sprintf(transmit, "DAC Value: %d | ADC Value : %d | Voltage: %2.2lfV \n",i , VoltagesRAWAvg[6], Voltages[6]);
		balancerwritetouart1(transmit);
		HAL_Delay(2000);
	}

}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  //MX_CAN1_Init();
  MX_DAC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

  balancerwritetouart1("Starting Balancer HAL...\n");
  startADCsampling();
  startTimers();
  disableAllBalancer();
  startDACoutput();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  //Converting RAW ADC Data to Voltages
	      calculateaveragevoltage();
	      convertRAWtoVoltage();
	      //convertRAWtoCurrent();
	      calcTemp();

	  //Print ADC Data to UART
	  //printRAWVoltages2UART1();
	      printabsRAWVoltages2UART1();

	      printVoltages2UART1();
	      calibrateVoltageInput();
	      //printRAWCurrents2UART1();
          //HAL_Delay(1000);

	  //sendcalibdatatoserial();
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
