/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "LCD.h"
#include "TS.h"
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COL_1			40
#define COL_2 		155
#define COL_3  	 	268
#define ROW_1			204
#define ROW_2			350
#define ROW_3			454
#define ERROR			25
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
unsigned char goalTemp[3][2];
unsigned char currTemp[3][2];
struct room {
	int goalTemp;
	int currTemp;
	int ID;
}typedef room_t;

void initRoom(room_t *room, int ID, int goalTemp, int currTemp) {
	room->ID = ID;
	room->goalTemp = goalTemp;
	room->currTemp = currTemp;
}

void initDisplay(SPI_HandleTypeDef *spi, room_t *rooms, int count) {
	LCD_writePixels(&hspi1, LCD_color565(255, 255, 255), 0, 0, LCD_WIDTH,
	LCD_HEIGHT);
	for (int i = 0; i < count; ++i) {
		itoa(rooms[i].goalTemp, &goalTemp[i][0], 10);
		itoa(rooms[i].currTemp, &currTemp[i][0], 10);

		int size = 8;
		int start = (LCD_WIDTH / 3) * (rooms[i].ID - 1) + size + 5;

		LCD_drawStringNoBG(spi, start, size, &currTemp[i][0], 2,
				LCD_color565(0, 0, 0), size);
		LCD_drawStringNoBG(spi, start, 9 * size, &goalTemp[i][0], 2,
				LCD_color565(75, 75, 75), size);

		LCD_drawButtonNoBG(&hspi1, start, 18 * size, 0, LCD_color565(255, 0, 0),
				size * 2 + 1);
		LCD_drawButtonNoBG(&hspi1, start, 34 * size, 1, LCD_color565(0, 0, 255),
				size * 2 + 1);
	}
	unsigned char heat[4] = "HEAT";
	unsigned char air[3] = "AIR";
	LCD_writePixels(&hspi1, LCD_color565(0, 0, 0), 6, LCD_HEIGHT - 8 * 6 - 4,
			23 * 6, 2);
	LCD_drawStringNoBG(spi, 6, LCD_HEIGHT - 8 * 6, &heat[0], 4,
			LCD_color565(0, 0, 0), 6);
	LCD_drawStringNoBG(spi, LCD_WIDTH - 18 * 6, LCD_HEIGHT - 8 * 6, &air[0], 3,
			LCD_color565(0, 0, 0), 6);
}

void updateDisplayTemp(SPI_HandleTypeDef *spi, room_t *room) {
	unsigned char buf[2];
	int size = 8;
	int start = (LCD_WIDTH / 3) * (room->ID - 1) + size + 5;

	itoa(room->currTemp, &buf[0], 10);
	if (buf[0] != currTemp[room->ID - 1][0]) {
		LCD_drawStringOPT(spi, start, size, &buf[0], &currTemp[room->ID - 1][0],
				2, LCD_color565(0, 0, 0), size);
		currTemp[room->ID - 1][0] = buf[0];
		currTemp[room->ID - 1][1] = buf[1];
	} else if (buf[1] != currTemp[room->ID - 1][1]) {
		LCD_drawCharOPT(spi, start + 6 * size, size, buf[1],
				currTemp[room->ID - 1][1], LCD_color565(0, 0, 0), size);
		currTemp[room->ID - 1][1] = buf[1];
	}

	itoa(room->goalTemp, &buf[0], 10);
	if (buf[0] != goalTemp[room->ID - 1][0]) {
		LCD_drawStringOPT(spi, start, 9 * size, &buf[0],
				&goalTemp[room->ID - 1][0], 2, LCD_color565(75, 75, 75), size);
		goalTemp[room->ID - 1][0] = buf[0];
		goalTemp[room->ID - 1][1] = buf[1];
	} else if (buf[1] != goalTemp[room->ID - 1][1]) {
		LCD_drawCharOPT(spi, start + 6 * size, 9 * size, buf[1],
				goalTemp[room->ID - 1][1], LCD_color565(75, 75, 75), size);
		goalTemp[room->ID - 1][1] = buf[1];
	}
}

void updateDisplayFurnace(SPI_HandleTypeDef *spi, int furnaceState) {
	int size = 6;
	if (furnaceState) {
		LCD_writePixels(spi, LCD_color565(255, 255, 255), size,
		LCD_HEIGHT - 8 * 6 - 4, 23 * size, 2);
		LCD_writePixels(&hspi1, LCD_color565(0, 0, 0), LCD_WIDTH - 18 * 6,
		LCD_HEIGHT - 8 * 6 - 4, 17 * size, 2);
	} else {
		LCD_writePixels(spi, LCD_color565(0, 0, 0), size,
		LCD_HEIGHT - 8 * 6 - 4, 23 * size, 2);
		LCD_writePixels(&hspi1, LCD_color565(255, 255, 255), LCD_WIDTH - 18 * 6,
		LCD_HEIGHT - 8 * 6 - 4, 17 * size, 2);
	}
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t curr_temp[] = { 0, 0, 0, 0 };
uint8_t desired_temp[] = { 0, 11, 11, 0 };

HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;

int interrupts_disabled = 0;
room_t rooms[3];

/* {0x00000000, 0x00000000} - ([0] => either 0 for desired state command, or a 1 for peripheral current_temp request command),
 *							([1] => first bit is furnace status, next 5 are desired temp, last 2 are vent id) */
uint8_t data_size = 2;
uint8_t received_data_size = 1;
uint8_t send_desired_state = 0x00;
uint8_t receive_current_temp = 0x80;
uint8_t furnace_status_on = 0x80;
uint8_t furnace_status_off = 0x00;
uint8_t vent_id_1 = 0x01;
uint8_t vent_id_2 = 0x02;
uint8_t vent_id_3 = 0x03;
uint8_t vent_ids[] = { 0x00, 0x01, 0x02, 0x03 };

HAL_StatusTypeDef transmit_updated_state(uint8_t data) {
	HAL_StatusTypeDef transmit_status = HAL_UART_Transmit(&huart4, data, 1,
			100);
	return transmit_status;
}

HAL_StatusTypeDef receive_acknowledge(uint8_t* received_data) {
	HAL_StatusTypeDef receive_status = HAL_UART_Receive(&huart4, received_data, 1, 100);
	return receive_status;
}

HAL_StatusTypeDef transmit_updated_state(room_t *rooms, int furnace_status) {
	HAL_StatusTypeDef transmit_status;
	uint8_t data[] = { send_desired_state, (uint8_t) ((furnace_status << 7)
			| ((rooms->goalTemp - 55) << 2) | rooms->ID) };
	for (int j = 0; j < sizeof(data); j++) {
		transmit_status = HAL_UART_Transmit(&huart4, &(data[j]), 1, 1000);
	}
	HAL_Delay(1000);
	return transmit_status;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_LPUART1_UART_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_UART4_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	int res = LCD_begin(&hspi1);
	for (int i = 0; i < 3; ++i)
		initRoom(&rooms[i], i + 1, 70, 65);
	initDisplay(&hspi1, &rooms[0], 3);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	int furnaceState = 0;
	double x, y, z;
	double X, Y;
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		uint8_t received_data[1];

		z = TS_readPressure(&hadc1);
		x = TS_readX(&hadc1);
		y = TS_readY(&hadc1);

		if (z < 1000) {
			transform(x, y, &X, &Y);
			int newGoal1 = 0, newGoal2 = 0, newGoal3 = 0, newFurnace = 0;
			if (X > COL_1 - ERROR && X < COL_1 + ERROR) {
				if (Y > ROW_1 - ERROR && Y < ROW_1 + ERROR) {
					rooms[0].goalTemp += 1;
					newGoal1 = 1;
				} else if (Y > ROW_2 - ERROR && Y < ROW_2 + ERROR) {
					rooms[0].goalTemp -= 1;
					newGoal1 = 1;
				} else if (Y > ROW_3 - ERROR && Y < ROW_3 + ERROR) {
					furnaceState = 0;
					newFurnace = 1;
				}
			} else if (X > COL_2 - ERROR && X < COL_2 + ERROR) {
				if (Y > ROW_1 - ERROR && Y < ROW_1 + ERROR) {
					rooms[1].goalTemp += 1;
					newGoal2 = 1;
				} else if (Y > ROW_2 - ERROR && Y < ROW_2 + ERROR) {
					rooms[1].goalTemp -= 1;
					newGoal2 = 1;
				}
			} else if (X > COL_3 - ERROR && X < COL_3 + ERROR) {
				if (Y > ROW_1 - ERROR && Y < ROW_1 + ERROR) {
					rooms[2].goalTemp += 1;
					newGoal3 = 1;
				} else if (Y > ROW_2 - ERROR && Y < ROW_2 + ERROR) {
					rooms[2].goalTemp -= 1;
					newGoal3 = 1;
				} else if (Y > ROW_3 - ERROR && Y < ROW_3 + ERROR) {
					furnaceState = 1;
					newFurnace = 1;
				}
			}

			for (int i = 0; i < 3; ++i) {
				rooms[i].goalTemp =
						(rooms[i].goalTemp < 99) ? rooms[i].goalTemp : 99;
				rooms[i].goalTemp =
						(rooms[i].goalTemp > -9) ? rooms[i].goalTemp : -9;
				rooms[i].currTemp =
						(rooms[i].currTemp < 99) ? rooms[i].currTemp : 99;
				rooms[i].currTemp =
						(rooms[i].currTemp > -9) ? rooms[i].currTemp : -9;
			}

			if (newGoal1) {
				interrupts_disabled = 1;
				while (receive_acknowledge(received_data) != HAL_OK) {
					transmit_updated_state(rooms[0].ID);
				}
				while (receive_acknowledge(received_data) != HAL_OK) {
					transmit_updated_state(rooms[0].goalTemp);
				}
				while (receive_acknowledge(received_data) != HAL_OK) {
					transmit_updated_state(furnaceState);
				}
				interrupts_disabled = 0;
				updateDisplayTemp(&hspi1, &rooms[0]);
			}
			if (newGoal2) {
				interrupts_disabled = 1;
				while (receive_acknowledge(received_data) != HAL_OK) {
					transmit_updated_state(rooms[1].ID);
				}
				while (receive_acknowledge(received_data) != HAL_OK) {
					transmit_updated_state(rooms[1].goalTemp);
				}
				while (receive_acknowledge(received_data) != HAL_OK) {
					transmit_updated_state(furnaceState);
				}
				interrupts_disabled = 0;
				updateDisplayTemp(&hspi1, &rooms[1]);
			}
			if (newGoal3) {
				interrupts_disabled = 1;
				while (receive_acknowledge(received_data) != HAL_OK) {
					transmit_updated_state(rooms[2].ID);
				}
				while (receive_acknowledge(received_data) != HAL_OK) {
					transmit_updated_state(rooms[2].goalTemp);
				}
				while (receive_acknowledge(received_data) != HAL_OK) {
					transmit_updated_state(furnaceState);
				}
				interrupts_disabled = 0;
				updateDisplayTemp(&hspi1, &rooms[2]);
			}
			if (newFurnace) {
				interrupts_disabled = 1;
				while (receive_acknowledge(received_data) != HAL_OK) {
					transmit_updated_state(rooms[0].ID);
				}
				while (receive_acknowledge(received_data) != HAL_OK) {
					transmit_updated_state(rooms[2].goalTemp);
				}
				while (receive_acknowledge(received_data) != HAL_OK) {
					transmit_updated_state(furnaceState);
				}
				interrupts_disabled = 0;
				updateDisplayFurnace(&hspi1, furnaceState);
			}
		}

//	  GPIO_PinState pin_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

//	  if(pin_state){
//		  for(int i = 0; i < sizeof(data); i++){
//			  transmit_status = HAL_UART_Transmit(&huart4, &(data[i]), 1, 1000);
//		  }
//		  transmit_status = HAL_UART_Transmit(&huart4, data, sizeof(data), 100);
//	  }

//	  for(int i = sizeof(received_data) - 1; i >= 0; i--){
//		  receive_status = HAL_UART_Receive(&huart4, &(received_data[i]), 1, 1000);
//	  }
//	  receive_status = HAL_UART_Receive(&huart4, received_data, sizeof(received_data), 100);

//	  if(received_data[0] == 31 && receive_status == HAL_OK){
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_SET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);
//	  } else {
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);
//	  }

//	  if(received_data[0] == 0x41){
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_SET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);
//	  } else if(received_data[0] == 0x42){
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_SET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_SET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);
//	  } else if(received_data[0] == 0x43){
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_SET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_SET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_SET);
//		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);
//	  } else {
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);
//	  }

		HAL_Delay(10);

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48
			| RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 2;
	RCC_OscInitStruct.PLL.PLLN = 30;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV12;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPUART1_UART_Init(void) {

	/* USER CODE BEGIN LPUART1_Init 0 */

	/* USER CODE END LPUART1_Init 0 */

	/* USER CODE BEGIN LPUART1_Init 1 */

	/* USER CODE END LPUART1_Init 1 */
	hlpuart1.Instance = LPUART1;
	hlpuart1.Init.BaudRate = 209700;
	hlpuart1.Init.WordLength = UART_WORDLENGTH_7B;
	hlpuart1.Init.StopBits = UART_STOPBITS_1;
	hlpuart1.Init.Parity = UART_PARITY_NONE;
	hlpuart1.Init.Mode = UART_MODE_TX_RX;
	hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
	if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN LPUART1_Init 2 */

	/* USER CODE END LPUART1_Init 2 */

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_EnableFifoMode(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 8399;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 49999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void) {

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
	hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
	hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
	if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	HAL_PWREx_EnableVddIO2();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PF12 PF13 PF14 */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
	GPIO_InitStruct.Pin = STLK_RX_Pin | STLK_TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : PD15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (!interrupts_disabled) {
		if (htim == &htim3) {
			uint8_t received_data[1];
			while (receive_acknowledge(received_data) != HAL_OK) {
				transmit_updated_state(1);
			}
			rooms[0].currTemp = received_data[0];
			while (receive_acknowledge(received_data) != HAL_OK) {
				transmit_updated_state(2);
			}
			rooms[1].currTemp = received_data[0];
			while (receive_acknowledge(received_data) != HAL_OK) {
				transmit_updated_state(3);
			}
			rooms[2].currTemp = received_data[0];
		}
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
