/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 09/10/2014 11:13:03
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

//test git

 
#include "example.h"
#include "example_usart.h"
#include "L6470.h"
#include <stdio.h>

int switchStatus;
int LEDStatus;
/**
  * @defgroup   MotionControl
  * @{
  */

/**
  * @addtogroup BSP
  * @{
  */

/**
  * @}
  */ /* End of BSP */

/**
  * @addtogroup MicrosteppingMotor_Example
  * @{
  */

/**
  * @defgroup   ExampleTypes
  * @{
  */

//#define MICROSTEPPING_MOTOR_EXAMPLE        //!< Uncomment to performe the standalone example
#define MICROSTEPPING_MOTOR_USART_EXAMPLE  //!< Uncomment to performe the USART example
#if ((defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option only!"
#elif ((!defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (!defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option!"
#endif
#if (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE) && (!defined (NUCLEO_USE_USART)))
  #error "Please define "NUCLEO_USE_USART" in "stm32fxxx_x-nucleo-ihm02a1.h"!"
#endif

/**
  * @}
  */ /* End of ExampleTypes */

/**
  * @brief The FW main module
  */
int main(void)
{
  switchStatus = 0;
  LEDStatus = 0;
  // int test; 
  // test = 1;
  /* NUCLEO board initialization */
  NUCLEO_Board_Init();
  
  /* X-NUCLEO-IHM02A1 initialization */
  BSP_Init();
  
#ifdef NUCLEO_USE_USART
  /* Transmit the initial message to the PC via UART */
  USART_TxWelcomeMessage();
  	USART_Transmit(&huart2, " X-CUBE-SPN2 v1.0.0\n\r");
#endif
  
#if defined (MICROSTEPPING_MOTOR_EXAMPLE)
  /* Perform a batch commands for X-NUCLEO-IHM02A1 */
  MicrosteppingMotor_Example_01();
  

#elif defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)
  /* Fill the L6470_DaisyChainMnemonic structure */
  Fill_L6470_DaisyChainMnemonic();
	
  // Input pin 4 
  GPIO_InitTypeDef PB4_Init_Struct;
  PB4_Init_Struct.Pin = GPIO_PIN_4;
  PB4_Init_Struct.Mode = GPIO_MODE_INPUT;
  PB4_Init_Struct.Pull = GPIO_PULLUP;
  PB4_Init_Struct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &PB4_Init_Struct);

//lab 2 interupt
// set up the GPIO interupt for pin 4
  GPIO_InitTypeDef GPIO_PIN_4_InitStruct;
  GPIO_PIN_4_InitStruct.Pin = GPIO_PIN_4;
  GPIO_PIN_4_InitStruct.Pull = GPIO_PULLUP;//GPIO_NOPULL;
  GPIO_PIN_4_InitStruct.Mode = GPIO_MODE_IT_FALLING; // For the normally closed limt switches
  HAL_GPIO_Init(GPIOB, &GPIO_PIN_4_InitStruct);
  
  /* Enable and set Button EXTI Interrupt to the lowest priority */
  HAL_NVIC_SetPriority((IRQn_Type)(EXTI4_IRQn), 0x0F, 0x00);
  HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI4_IRQn)); // enable interupt on pin 4

// PB 6
  GPIO_InitTypeDef PB6_Init_Struct;
  PB6_Init_Struct.Pin = GPIO_PIN_6;
  PB6_Init_Struct.Mode = GPIO_MODE_INPUT;
  PB6_Init_Struct.Pull = GPIO_PULLUP;
  PB6_Init_Struct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &PB6_Init_Struct);

// set up the GPIO interupt for pin 6
  GPIO_InitTypeDef GPIO_PIN_6_InitStruct;
  GPIO_PIN_6_InitStruct.Pin = GPIO_PIN_6;
  GPIO_PIN_6_InitStruct.Pull = GPIO_PULLUP;//GPIO_NOPULL;
  // GPIO_PIN_6_InitStruct.Mode = GPIO_MODE_IT_RISING; 
  GPIO_PIN_6_InitStruct.Mode = GPIO_MODE_IT_FALLING; // For the normally closed limt switches
  HAL_GPIO_Init(GPIOB, &GPIO_PIN_6_InitStruct);

  /* Enable and set Button EXTI Interrupt to the lowest priority */
  HAL_NVIC_SetPriority((IRQn_Type)(EXTI9_5_IRQn), 0x0F, 0x00);
  HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI9_5_IRQn)); // enable interupt on pin 4

// Output pin PB 10
  GPIO_InitTypeDef PB10_Init_Struct;
  PB10_Init_Struct.Pin = GPIO_PIN_10;
  PB10_Init_Struct.Mode = GPIO_MODE_INPUT;
  PB10_Init_Struct.Pull = GPIO_PULLUP;
  PB10_Init_Struct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &PB10_Init_Struct);

  // set up the GPIO interupt for pin 10
  GPIO_InitTypeDef GPIO_PIN_10_InitStruct;
  GPIO_PIN_10_InitStruct.Pin = GPIO_PIN_10;
  GPIO_PIN_10_InitStruct.Pull = GPIO_PULLUP;//GPIO_NOPULL;
  // GPIO_PIN_6_InitStruct.Mode = GPIO_MODE_IT_RISING; 
  GPIO_PIN_10_InitStruct.Mode = GPIO_MODE_IT_FALLING; // For the normally closed limt switches
  HAL_GPIO_Init(GPIOB, &GPIO_PIN_10_InitStruct);

  /* Enable and set Button EXTI Interrupt to the lowest priority */
  HAL_NVIC_SetPriority((IRQn_Type)(EXTI15_10_IRQn), 0x0F, 0x00);
  HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI15_10_IRQn)); // enable interupt on pin 4

  // set up the GPIO interupt for pin PB13
  GPIO_InitTypeDef GPIO_PIN_5_InitStruct;
  GPIO_PIN_5_InitStruct.Pin = GPIO_PIN_5;
  GPIO_PIN_5_InitStruct.Pull = GPIO_PULLUP;
  GPIO_PIN_5_InitStruct.Mode = GPIO_MODE_IT_FALLING; // For the normally closed limt switches
  HAL_GPIO_Init(GPIOA, &GPIO_PIN_5_InitStruct);

  HAL_NVIC_SetPriority((IRQn_Type)(EXTI9_5_IRQn), 0x0F, 0x00);
  HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI9_5_IRQn));

  // ADC Pin setup
  GPIO_InitTypeDef PB0_Init_Struct;
  PB0_Init_Struct.Pin = GPIO_PIN_0;
  PB0_Init_Struct.Mode = GPIO_MODE_ANALOG;
  PB0_Init_Struct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &PB0_Init_Struct);

  


    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
__GPIOB_CLK_ENABLE();

  //  For single channel conversion
  //ADC_HandleTypeDef hadc1;
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
   
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);


	/*Initialize the motor parameters */
	Motor_Param_Reg_Init();
  
  // L6470_Run(0, L6470_DIR_REV_ID, 1000); //LEFT MOTOR X 
  // L6470_Run(1, L6470_DIR_REV_ID, 1000); //LEFT MOTOR Y
  /* Infinite loop */
  uint32_t adc_out;
  uint32_t speed;
  while (1)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    adc_out = HAL_ADC_GetValue(&hadc1);


    // Y-axis reverse
    // if (adc_out <= 0x64)
    if (adc_out <= 100)
    {
      speed = 10000 - 90*adc_out ;
      L6470_Run(1, L6470_DIR_REV_ID, speed); 
      // USART_Transmit(&huart2, "Y-axis reverse\n\r");
    }
    // Y dead zone
    // else if (adc_out > 0x64 && adc_out < 0x9C)
    else if (adc_out > 100 && adc_out < 156)
    {
      speed = 0;
      L6470_Run(1, L6470_DIR_REV_ID, speed);
      // USART_Transmit(&huart2, "Y-axis dead zone\n\r");
    }
    // Y-axis forward
    // else if (adc_out >= 0x9C)
    else if (adc_out >= 156)
    {
      speed = 90*(adc_out-155) + 1000;
      L6470_Run(1, L6470_DIR_FWD_ID, speed);
      // USART_Transmit(&huart2, "Y-axis forward\n\r");
    }
    // USART_Transmit(&huart2, num2hex(speed, DOUBLEWORD_F));

    
    // USART_Transmit(&huart2, ('%d',adc_out))
    // USART_Transmit(&huart2, num2hex(speed, DOUBLEWORD_F));
    // USART_Transmit(&huart2, "\n\r");
  
    // USART_Transmit(&huart2, num2hex(adc_out, DOUBLEWORD_F));
    // USART_Transmit(&huart2, "\n\r");


    HAL_ADC_Stop(&hadc1);
    
    // adc_out = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);

    
    
    // LEDStatus = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10); // LED output 

    /* Check if any Application Command for L6470 has been entered by USART */
    USART_CheckAppCmd();
  }
#endif
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


// MOTOR FUNCTIONS

// void motor_Run(direction, axis)
// {

//   StepperMotorBoardHandle->Command->Move(board, device, L6470_DIR_FWD_ID, Step)
//   /* Enable the L6470 powerstage */
//   // BSP_MotorControl_CmdEnable(0);
//   /* Infinite loop */
//   while(1);
// }

// void motor_Reverse(void)
// {
//   /* Reverse the motor */
//   // BSP_MotorControl_SetParam(0, L6470_ABS_POS, 0);
//   // BSP_MotorControl_Move(0, BACKWARD, 0);
//   // /* Infinite loop */
//   // while(1);
// }

// void motor_Stop(void)
// {
  
  
// }





/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/**
  * @}
  */ /* End of MotionControl */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
