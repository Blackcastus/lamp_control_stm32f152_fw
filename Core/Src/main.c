/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sh1106.h" // Thư viện OLED SH1106
#include "INA219.h"
#include <stdio.h>  // �?ể sử dụng snprintf
#include <string.h> // �?ể sử dụng strlen
#include "bitmaps.h"
#include "processing_handle.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
INA219_t ina219;
/* USER CODE BEGIN PV */

const uint8_t mValid_Uid[14] = {0x02, 0x31, 0x31, 0x30, 0x30, 0x33, 0x41, 0x34, 0x31, 0x33, 0x45, 0x35, 0x34, 0x03};

uint8_t mButton_Sos_Pressed = 0; // status button SOS
uint8_t mButton_Rfid_Pressed = 0; // Status button On/Off power read RFID
uint8_t mRfidBuffer[14];               // Bộ đệm nhận dữ liệu từ module RFID
uint16_t mVotage_Value = 0;
uint16_t mCurrent_Value = 0;
uint8_t mBattery_percent = 0;
uint8_t mCharging_status = 0; // default 0 is not charging

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

static void DISPLAY_PROCESS(void);
static void RFID_PROCESS(void);
static void SOS_PROCESS(void);
static void BATERRY_CHARGER_PROCESS(void);

static int CompareData(const uint8_t *received_data, const uint8_t *valid_data);


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
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  if (!INA219_Init(&ina219, &hi2c2, INA219_ADDRESS)) {
	 while (1);
 }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

   SH1106_Init();
   HAL_Delay(100);
   SH1106_Fill(SH1106_COLOR_BLACK);

   SH1106_DrawBitmap(45, 0, (char*)bmp_logo_ontrak.data,bmp_logo_ontrak.width, bmp_logo_ontrak.height, SH1106_COLOR_WHITE);

   SH1106_UpdateScreen();
   HAL_Delay(2000);

   while (1) {
	   BATERRY_CHARGER_PROCESS();
	   DISPLAY_PROCESS();
	   SOS_PROCESS();
	   RFID_PROCESS();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, COI_Pin|RDM6300_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COI_Pin RDM6300_Pin */
  GPIO_InitStruct.Pin = COI_Pin|RDM6300_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin B2_Pin */
  GPIO_InitStruct.Pin = B1_Pin|B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * Display process
 */
static void DISPLAY_PROCESS(void)
{
   static refresh_display_time = 0;
   char vbatBuffer[16];
   char ibatBuffer[16];
   char perceent_batBuffer[16];


   snprintf(vbatBuffer, sizeof(vbatBuffer), "VBat: %0.3f", mVotage_Value/1000.0); // Hiển thị vbat
   snprintf(ibatBuffer, sizeof(ibatBuffer), "IBat: %d mA", mCurrent_Value);            // Hiển thị ibat
   snprintf(perceent_batBuffer, sizeof(perceent_batBuffer), "%d%%", mBattery_percent);

   SH1106_Fill(SH1106_COLOR_BLACK);
   uint8_t x = 96;
   uint8_t y = 0;
   uint8_t bat_width = bmp_bat_0_percent.width;
   uint8_t bat_height = bmp_bat_0_percent.height;
   if((HAL_GetTick() - refresh_display_time) >= 100)
	{

	   refresh_display_time = HAL_GetTick();

	   if(mBattery_percent < 20)
	   {
		   SH1106_DrawBitmap(x, y, (char*)bmp_bat_0_percent.data,bat_width, bat_height, SH1106_COLOR_WHITE); // 0 - 20%
	   }
	   else if(mBattery_percent < 40)
	   {
		   SH1106_DrawBitmap(x, y, (char*)bmp_bat_30_percent.data,bat_width, bat_height, SH1106_COLOR_WHITE); // 20 - 40%
	   }
	   else if(mBattery_percent < 70)
	   {
		   SH1106_DrawBitmap(x, y, (char*)bmp_bat_50_percent.data,bat_width, bat_height, SH1106_COLOR_WHITE); // 40 - 70%
	   }
	   else if(mBattery_percent < 90)
	   {
		   SH1106_DrawBitmap(x, y, (char*)bmp_bat_80_percent.data,bat_width, bat_height, SH1106_COLOR_WHITE); // 70 - 90%
	   }
	   else
	   {
		   SH1106_DrawBitmap(x, y, (char*)bmp_bat_100_percent.data,bat_width, bat_height, SH1106_COLOR_WHITE);  // 90 - 100%
	   }

//	   Display Charging status
	   if(mCharging_status)
	   {
		   SH1106_DrawBitmap(60, 0, (char*)bmp_bat_charging.data,bmp_bat_charging.width, bmp_bat_charging.height, SH1106_COLOR_WHITE);
	   }

//	   SH1106_GotoXY(5, 0); // the first line
//	   SH1106_Puts(vbatBuffer, &Font_7x10, SH1106_COLOR_WHITE);
	   SH1106_GotoXY(70, 4); // the first line
	   SH1106_Puts(perceent_batBuffer, &Font_7x10, SH1106_COLOR_WHITE);

	   SH1106_GotoXY(5, 20); // the second line
	   if (mButton_Sos_Pressed)
	   {
		   SH1106_Puts("SOS: ON", &Font_7x10, SH1106_COLOR_WHITE);
	   } else {
		   SH1106_Puts("SOS: OFF", &Font_7x10, SH1106_COLOR_WHITE);
	   }

	   SH1106_GotoXY(5, 30); // the third line
	   if(mButton_Rfid_Pressed)
	   {
		   SH1106_Puts("RFID: ON", &Font_7x10, SH1106_COLOR_WHITE);
		   if(mCharging_status)
		   {
			   SH1106_GotoXY(5, 40); // the third line
			   SH1106_Puts("Battery Charging", &Font_7x10, SH1106_COLOR_WHITE);
		   }
	   }
	   else
	   {
		   SH1106_Puts("RFID: OFF", &Font_7x10, SH1106_COLOR_WHITE);
		   mCharging_status = 0;
	   }

	   SH1106_UpdateScreen();
	}
}

/**
 * RFID process
 */
static void RFID_PROCESS(void)
{
	static rfid_time_last = 0;
 if(mButton_Rfid_Pressed)
 {
	 RFID_CONTROL(TURN_ON);

	 if((HAL_GetTick() - rfid_time_last ) > 100)
	{
		 rfid_time_last = HAL_GetTick();
		 HAL_UART_Receive(&huart1, mRfidBuffer, 14, 100);
		 if(CompareData(mRfidBuffer, mValid_Uid))
		 {
			 mCharging_status = 1;

		 }
		 else
		 {
			 mCharging_status = 0;
		 }
	}
 }
 else
 {
	 RFID_CONTROL(TURN_OFF);
 }
}

/**
 * Checking button sos process
 */
static void SOS_PROCESS(void)
{
	if(mButton_Sos_Pressed)
	{
		BUZZER_CONTROL(TURN_ON);
	}
	else
	{
		BUZZER_CONTROL(TURN_OFF);
	}
}

/**
 * baterry charger process
 */
static void BATERRY_CHARGER_PROCESS(void)
{
   uint16_t vbus_mV = INA219_ReadBusVoltage(&ina219); // Đọc Vbus (mV)
   uint16_t icurrent_mV = INA219_ReadCurrent(&ina219); // Đọc dòng điện (mA)

   mVotage_Value = vbus_mV;
   mCurrent_Value = icurrent_mV;
   float tmp = mVotage_Value/BATTERY_FULL;
   mBattery_percent = tmp * 100;

   if(mCharging_status)
   {
	   if((mVotage_Value) >= BATTERY_FULL)
	   {
		   // charging stop
	   }
	   else
	   {
		   // charging start
	   }
   }
   else
   {
	   // charging stop
   }
}

/**
 * Compare Data matching or no matching
 */
static int CompareData(const uint8_t *received_data, const uint8_t *valid_data)
{
	uint8_t status = 1; // default matching
    for (int i = 0; i < 14; i++)
    {
        if (received_data[i] != valid_data[i])
        {
        	status = 0; // No matching
        	break;
        }
    }
    return status;
}

/**
 * Callback
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	UNUSED(GPIO_Pin);
    if (GPIO_Pin == B1_Pin)
    { // Nút nhấn B1
    	__HAL_GPIO_EXTI_CLEAR_IT(B1_Pin);
    	mButton_Sos_Pressed ^= 1;
    }
    else if (GPIO_Pin == B2_Pin)
    { // Nút nhấn B2
    	__HAL_GPIO_EXTI_CLEAR_IT(B2_Pin);
    	mButton_Rfid_Pressed ^= 1;
    }
    else
    {
//    	_NOP();
    }
}

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
