/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
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

/* USER CODE BEGIN PV */
FDCAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];
uint8_t canDatacheck = 0;

FDCAN_TxHeaderTypeDef txHeader;
uint8_t txData[8];

// looks like my scanners need that to detect presence of a vehicle
uint8_t pids_supported_01_20_service_01[8] =
{ 0x06, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA };

// https://www.csselectronics.com/pages/obd2-pid-table-on-board-diagnostics-j1979
// https://en.wikipedia.org/wiki/OBD-II_PIDs
uint8_t pids_supported_service_09[8] =
{ 0x06, 0x49, 0x00, 0x60, 0x00, 0x00, 0x00, 0xAA }; // VIN support
// https://en.wikipedia.org/wiki/Vehicle_identification_number
char vin[] = "L0V3Y0UKAT3NJUL1A"; // 17 characters

uint32_t frame_separation_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SendCanFrame(void);
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
	MX_FDCAN1_Init();
	/* USER CODE BEGIN 2 */
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			0) != HAL_OK)
	{
		Error_Handler();
	}

	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_8;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		if (canDatacheck == 1)
		{
			canDatacheck = 0;

			if ((rxHeader.Identifier == 0x7DF) && (rxData[0] == 0x02)
					&& (rxData[1] == 0x01) && (rxData[2] == 0x00)) // Welcome PIDs 0x01-0x20 Service 01
			{
				txHeader.Identifier = 0x7E8;
				memcpy(txData, pids_supported_01_20_service_01, 8);
				SendCanFrame();
				HAL_GPIO_TogglePin(USER_BLUE_LED_GPIO_Port, USER_BLUE_LED_Pin);
			}
			else if ((rxHeader.Identifier == 0x7DF) && (rxData[0] == 0x02)
					&& (rxData[1] == 0x09) && (rxData[2] == 0x00)) // Welcome PIDs Service 09
			{
				txHeader.Identifier = 0x7E8;
				memcpy(txData, pids_supported_service_09, 8);
				SendCanFrame();
				HAL_GPIO_TogglePin(USER_BLUE_LED_GPIO_Port, USER_BLUE_LED_Pin);
			}
			else if ((rxHeader.Identifier == 0x7DF) && (rxData[0] == 0x02)
					&& (rxData[1] == 0x09) && (rxData[2] == 0x02)) // VIN
			{
				// Frame 1
				txHeader.Identifier = 0x7E8;
				// https://en.wikipedia.org/wiki/ISO_15765-2
				// https://automotivevehicletesting.com/vehicle-diagnostics/uds-protocol/iso-15765-2-protocol/
				// https://community.carloop.io/t/how-to-request-vin/153/4
				// https://www.csselectronics.com/pages/obd2-explained-simple-intro (Example 1)
				txData[0] = 0x10; // [first nibble/nybble of first byte] PCI (Protocol Control Information): first frame of a multi-frame/extended message
				txData[1] = 0x14; // [second nibble/nybble of first byte and entire second byte] payload size: 20 bytes
				txData[2] = 0x49; // Response (0x40) for service 0x09
				txData[3] = 0x02; // PID for VIN
				txData[4] = 0x01; // NODI (no data indicator)
				txData[5] = vin[0]; // VIN first byte
				txData[6] = vin[1];
				txData[7] = vin[2];
				SendCanFrame();
			}
			else if ((rxHeader.Identifier == 0x7E8 - 0x008)
					&& (rxData[0] == 0x30) && (rxData[1] == 0x00)) // control frame
			{
				frame_separation_time = rxData[2]; // ms
				// Frame 2
				txHeader.Identifier = 0x7E8;
				txData[0] = 0x21; // Continuation frame identifier [PCI and seq num]
				txData[1] = vin[3];
				txData[2] = vin[4];
				txData[3] = vin[5];
				txData[4] = vin[6];
				txData[5] = vin[7];
				txData[6] = vin[8];
				txData[7] = vin[9];
				HAL_Delay(frame_separation_time);
				SendCanFrame();
				// Frame 3
				txHeader.Identifier = 0x7E8;
				txData[0] = 0x22; // Continuation frame identifier [PCI and seq num]
				txData[1] = vin[10];
				txData[2] = vin[11];
				txData[3] = vin[12];
				txData[4] = vin[13];
				txData[5] = vin[14];
				txData[6] = vin[15];
				txData[7] = vin[16]; // VIN last byte
				HAL_Delay(frame_separation_time);
				SendCanFrame();
				HAL_GPIO_TogglePin(USER_GREEN_LED_GPIO_Port,
				USER_GREEN_LED_Pin);
			}
			else
			{
				__NOP();
			}
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData)
				!= HAL_OK)
		{
			Error_Handler();
		}
		else
		{
			canDatacheck = 1;
		}
	}
}

void SendCanFrame(void)
{
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) != HAL_OK)
	{
		Error_Handler();
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
