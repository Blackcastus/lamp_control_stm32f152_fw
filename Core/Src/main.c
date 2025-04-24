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
float 	mVotage_Value = 0.0;
int16_t mCurrent_Value = 0;
uint8_t mStatus_Button_Sos = 0; // default

uint8_t mTemp_cnt = 0;
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
//   SH1106_GotoXY(30, 25);
//   SH1106_Puts("OnTrak", &Font_11x18, SH1106_COLOR_WHITE);
   SH1106_UpdateScreen();
//   HAL_Delay(2000);

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
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
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
   char vbatBuffer[16];
   char ibatBuffer[16];
   char tempcnt[16];

   snprintf(vbatBuffer, sizeof(vbatBuffer), "VBat: %0.3f V", mVotage_Value); // Hiển thị vbat
   snprintf(ibatBuffer, sizeof(ibatBuffer), "IBat: %d mA", mCurrent_Value);            // Hiển thị ibat
   snprintf(tempcnt, sizeof(tempcnt), "Cnt: %d ", mTemp_cnt);

   SH1106_Fill(SH1106_COLOR_BLACK);
   SH1106_GotoXY(5, 0); // the first line
   SH1106_Puts(vbatBuffer, &Font_7x10, SH1106_COLOR_WHITE);
   SH1106_GotoXY(5, 10); // the second line
   SH1106_Puts(ibatBuffer, &Font_7x10, SH1106_COLOR_WHITE);
   SH1106_GotoXY(5, 20); // the third line
   SH1106_Puts(tempcnt, &Font_7x10, SH1106_COLOR_WHITE);
//   if (mStatus_Button_Sos)
//   {
//	   SH1106_Puts("SOS: ON", &Font_7x10, SH1106_COLOR_WHITE);
//   } else {
//	   SH1106_Puts("SOS: OFF", &Font_7x10, SH1106_COLOR_WHITE);
//   }

   SH1106_UpdateScreen();
   HAL_Delay(100);
}

/**
 * RFID process
 */
static void RFID_PROCESS(void)
{
	// Quét RFID liên tục
		   if (HAL_UART_Receive(&huart1, mRfidBuffer, 14, 100) == HAL_OK) { // Nhận dữ liệu RFID
				  if (CompareData(mRfidBuffer, mValid_Uid)) {
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // Bật LED
						SH1106_GotoXY(5, 30); // Cập nhật dòng 3
						SH1106_Puts("charger", &Font_7x10, SH1106_COLOR_WHITE);
	//         	            SH1106_UpdateScreen();
				  }
				  else
				  {
					 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // Tắt LED
				}
				   HAL_Delay(10);
			  }

		   // Kiểm tra nút B2 để bật/tắt nguồn RFID
		   if (HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin) == GPIO_PIN_RESET) { // Kiểm tra nếu nút B2 được nhấn
			   HAL_Delay(50); // Chống dội nút (Debounce)
			   if (HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin) == GPIO_PIN_RESET) { // Xác nhận nút vẫn được nhấn
				   static uint8_t rdm6300_state = 0; // Trạng thái nguồn RFID
				   rdm6300_state = !rdm6300_state; // Đảo trạng thái nguồn
				   HAL_GPIO_WritePin(GPIOC, RDM6300_Pin, rdm6300_state ? GPIO_PIN_SET : GPIO_PIN_RESET); // Điều khiển nguồn RFID

				   // Hiển thị trạng thái bật/tắt nguồn RFID trên OLED
				   SH1106_GotoXY(5, 30); // Dòng thứ ba
				   if (rdm6300_state) {
					   SH1106_Puts("RFID ON", &Font_7x10, SH1106_COLOR_WHITE);
				   } else {
					   SH1106_Puts("RFID OFF", &Font_7x10, SH1106_COLOR_WHITE);
	//                       is_charger_displayed = 0; // Xóa trạng thái hiển thị khi tắt nguồn RFID
				   }
	//                   SH1106_UpdateScreen();

				   // Chờ thả nút
				   while (HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin) == GPIO_PIN_RESET);
			   }
		   }
}

/**
 * Checking button sos process
 */
static void SOS_PROCESS(void)
{
////   // Kiểm tra nút B1
//   if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) { // Kiểm tra nếu nút B1 được nhấn
//	   HAL_Delay(50); // Chống dội nút (Debounce)
//	   if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) { // Xác nhận nút vẫn được nhấn
//		   static uint8_t led_b1_state = 0; // Trạng thái LED cho nút B1
//		   led_b1_state = !led_b1_state; // Đảo trạng thái LED
//		   HAL_GPIO_WritePin(GPIOC, COI_Pin, led_b1_state ? GPIO_PIN_SET : GPIO_PIN_RESET); // Điều khiển LED
//
//		   // Hiển thị trạng thái B1 lên OLED
//		   SH1106_GotoXY(5, 40); // Dòng thứ ba
//		   if (led_b1_state) {
//			   SH1106_Puts("SOS: ON", &Font_7x10, SH1106_COLOR_WHITE);
//		   } else {
//			   SH1106_Puts("SOS: OFF", &Font_7x10, SH1106_COLOR_WHITE);
//		   }
//
//		   // Chờ thả nút
//		   while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET);
//	   }
//   }
}

/**
 * baterry charger process
 */
static void BATERRY_CHARGER_PROCESS(void)
{
	// Đọc giá trị từ INA219
   uint16_t vbus_mV = INA219_ReadBusVoltage(&ina219); // Đọc Vbus (mV)
   mCurrent_Value = INA219_ReadCurrent(&ina219); // Đọc dòng điện (mA)

   // Chia tách phần nguyên và phần thập phân
   mVotage_Value = vbus_mV / 1000.0;          // Phần nguyên của Vbat
//   mCurrent_Value = current_mA / 1000;         // Phần thập phân của Vbat

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
    if (GPIO_Pin == B1_Pin)
    { // Nút nhấn B1
        mButton_Rfid_Pressed != mButton_Rfid_Pressed; // Cờ trạng thái nút nhấn B1
    }
    else if (GPIO_Pin == B2_Pin)
    { // Nút nhấn B2
    	mStatus_Button_Sos != mStatus_Button_Sos;
    	mTemp_cnt++;
//        mButton_Sos_Pressed = 1; // Cờ trạng thái nút nhấn B2
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
