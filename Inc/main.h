/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define IBZ4_Pin GPIO_PIN_0
#define IBZ4_GPIO_Port GPIOC
#define IBZ3_Pin GPIO_PIN_1
#define IBZ3_GPIO_Port GPIOC
#define IBZ2_Pin GPIO_PIN_2
#define IBZ2_GPIO_Port GPIOC
#define IBZ1_Pin GPIO_PIN_3
#define IBZ1_GPIO_Port GPIOC
#define UZ6_Pin GPIO_PIN_0
#define UZ6_GPIO_Port GPIOA
#define UZ5_Pin GPIO_PIN_1
#define UZ5_GPIO_Port GPIOA
#define UZ4_Pin GPIO_PIN_2
#define UZ4_GPIO_Port GPIOA
#define UZ3_Pin GPIO_PIN_3
#define UZ3_GPIO_Port GPIOA
#define DAC1_Pin GPIO_PIN_4
#define DAC1_GPIO_Port GPIOA
#define UZ1_Pin GPIO_PIN_5
#define UZ1_GPIO_Port GPIOA
#define UBATSENSE_Pin GPIO_PIN_6
#define UBATSENSE_GPIO_Port GPIOA
#define IBATSENSE_Pin GPIO_PIN_4
#define IBATSENSE_GPIO_Port GPIOC
#define UZ2_Pin GPIO_PIN_5
#define UZ2_GPIO_Port GPIOC
#define IBZ6_Pin GPIO_PIN_0
#define IBZ6_GPIO_Port GPIOB
#define IBZ5_Pin GPIO_PIN_1
#define IBZ5_GPIO_Port GPIOB
#define Cell1PWM_Pin GPIO_PIN_9
#define Cell1PWM_GPIO_Port GPIOE
#define Cell2PWM_Pin GPIO_PIN_11
#define Cell2PWM_GPIO_Port GPIOE
#define Cell3PWM_Pin GPIO_PIN_13
#define Cell3PWM_GPIO_Port GPIOE
#define Cell4PWM_Pin GPIO_PIN_14
#define Cell4PWM_GPIO_Port GPIOE
#define ENGCON_Pin GPIO_PIN_10
#define ENGCON_GPIO_Port GPIOD
#define D_C_LCD_Pin GPIO_PIN_11
#define D_C_LCD_GPIO_Port GPIOD
#define RST_LCD_Pin GPIO_PIN_12
#define RST_LCD_GPIO_Port GPIOD
#define CS_SD_Pin GPIO_PIN_13
#define CS_SD_GPIO_Port GPIOD
#define CS_LCD_Pin GPIO_PIN_14
#define CS_LCD_GPIO_Port GPIOD
#define CS_TOUCH_Pin GPIO_PIN_15
#define CS_TOUCH_GPIO_Port GPIOD
#define Cell5PWM_Pin GPIO_PIN_6
#define Cell5PWM_GPIO_Port GPIOC
#define Cell6PWM_Pin GPIO_PIN_7
#define Cell6PWM_GPIO_Port GPIOC
#define ENFAN_Pin GPIO_PIN_8
#define ENFAN_GPIO_Port GPIOC
#define Touch_IRQ_Pin GPIO_PIN_2
#define Touch_IRQ_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
