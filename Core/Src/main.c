/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include "LoRa.h"
#include "DFRobot_GNSS.h"
#include "ili9486.h"
#include "fonts.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define GRID_STEP 40
#define SCREEN_REFRESH_PERIOD 1000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef enum {
	INIT,
	TX_RX,
	DRAW_SATS,
	DRAW_FAILED,
	DRAW_RSSI,
	PLOT,
} SystemState;



LoRa myLoRa;
DFRobot_GNSS_t gnss;

// DATA FOR TRANSCEIVER
uint8_t received_data[64];
uint8_t packet_size = 0;
char msg_buffer[65];
	
char* send_data;
	
sGNSS_Data_t currentData;
char lora_payload[64];

char message[100];

void print_msg(char *msg) {
	HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
}

/**
 * @brief Prints the current system state and GNSS info to UART.
 * @param state The current SystemState of the FSM
 * @param sats  The number of satellites currently visible
 */
void Log_System_Status(SystemState state, int sats) {
    char state_name[20];

    // Convert Enum to String for printing
    switch (state) {
        case INIT:         strcpy(state_name, "INIT"); break;
        case TX_RX:        strcpy(state_name, "TX_RX"); break;
        case DRAW_SATS:    strcpy(state_name, "DRAW_SATS"); break;
        case DRAW_FAILED:  strcpy(state_name, "DRAW_FAILED"); break;
        case DRAW_RSSI:    strcpy(state_name, "DRAW_RSSI"); break;
        case PLOT:         strcpy(state_name, "PLOT"); break;
        default:           strcpy(state_name, "UNKNOWN"); break;
    }

    // Format the message with both the state name and the satellite count
    // Using \r\n ensures compatibility with most serial monitors (Putty, TeraTerm)
    sprintf(message, "\r\n[STATE: %s] - Sats visible: %d\r\n", state_name, sats);
    print_msg(message);
}

/**
 * @brief  Initializes peripherals.
 * @return 1 if all peripherals initialized successfully, 0 if any failed.
 */
uint8_t setup_peripherals(void) {
    uint8_t status = 1; // Assume success initially

    sprintf(message, "\nStarting\n");
    print_msg(message);
    
    // --- DISPLAY SETUP ---
    ILI9486_Init();
    ILI9486_SetRotation(1);
    ILI9486_FillScreen(ILI9486_ColorRGB(0, 0, 0)); 
    ILI9486_DrawString(10, 10, "Starting Peter Long Range...", Font_11x18, WHITE, BLACK);
    
    // --- LORA SETUP ---
    myLoRa = newLoRa();
    myLoRa.CS_port    = LoRa_NSS_GPIO_Port;
    myLoRa.CS_pin     = LoRa_NSS_Pin;
    myLoRa.reset_port = LoRa_RESET_GPIO_Port;
    myLoRa.reset_pin  = LoRa_RESET_Pin;
    myLoRa.DIO0_port  = DIO0_GPIO_Port;
    myLoRa.DIO0_pin   = DIO0_Pin;
    myLoRa.hSPIx      = &hspi1;

    if (LoRa_init(&myLoRa) == LORA_OK) {
        sprintf(message, "LoRa Ready\r\n");
        print_msg(message);
        ILI9486_DrawString(10, 40, "LoRa Ready", Font_11x18, MAGENTA, BLACK);
    } else {
        sprintf(message, "Error in LoRa connection\r\n");
        print_msg(message);
        ILI9486_DrawString(10, 40, "Error in LoRa connection", Font_11x18, RED, BLACK);
        status = 0; // Mark failure
    }
    
    // --- GNSS SETUP ---
    GNSS_Init(&gnss, &hi2c2); 
    GNSS_PowerControl(&gnss, true);
    
    // GNSS sensor handshake
    if (HAL_I2C_IsDeviceReady(&hi2c2, GNSS_DEVICE_ADDR, 1, 20000) == HAL_OK) {
        GNSS_SetMode(&gnss, eGPS_BeiDou_GLONASS);
        sprintf(message, "GNSS Ready");
        print_msg(message);
        ILI9486_DrawString(10, 70, "GNSS Ready", Font_11x18, GREEN, BLACK);
    } else {
        sprintf(message, "Error in GNSS connection");
        print_msg(message);
        ILI9486_DrawString(10, 70, "Error in GNSS connection", Font_11x18, RED, BLACK);
        status = 0; // Mark failure
    }

    HAL_Delay(1000);
		

    return status;
}

void GNSS_get_data() {
	
	// get data fromt gnss
	currentData = GNSS_GetAllData(&gnss);
	
	// print number of sats visible
	sprintf(message, "\nSats visible: %d\r\n", currentData.satellites);
	print_msg(message);
	
	// print coordinates
	sprintf(message, "Lat: %f %c, Lon: %f %c\r\n", 
					currentData.latitude, currentData.latDirection, 
					currentData.longitude, currentData.lonDirection);
	print_msg(message);
				
	// put the gnss data into lora_payload
	snprintf(lora_payload, sizeof(lora_payload), // snprintf specifies size
						"\nLAT:%.6f%c,LON:%.6f%c",
						currentData.latitude,
						currentData.latDirection,
						currentData.longitude,
						currentData.lonDirection);
}

void LoRa_Transmit() {
	//START TRANSMITTING
	if(LoRa_transmit(&myLoRa, (uint8_t*)lora_payload, strlen(lora_payload), 100) == 1) {
		print_msg("\nTransmission Successful!\r");
	} else {
		print_msg("\nTransmission Failed.\r");
	}
}

uint8_t LoRa_Recieve() {
	//START RECEIVE
	LoRa_startReceiving(&myLoRa);
	packet_size = LoRa_receive(&myLoRa, received_data, sizeof(received_data));
		
	if (packet_size > 0) {		
		//PRINT DECODED DATA
		memcpy(msg_buffer, received_data, packet_size);
		msg_buffer[packet_size] = '\0';
		
		sprintf(message, "\nReceived Coordinates: %s", msg_buffer);
		print_msg(message);
		return 1; // return 1 if success
	}
	return 0; // return 0 if unsuccessful
}

void calculate_position() {}
	
void draw_position() {}

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
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	SystemState currentState = INIT;
	
	uint32_t refreshedTime = HAL_GetTick();
	uint8_t dataRecieved = 0;
	
  while (1) {
		// FSM
		switch (currentState) {
			case INIT: {
				uint8_t setupStatus = setup_peripherals();
				
				if (setupStatus) {
					// Draw the background
					ILI9486_DrawRadarGrid(GRID_STEP);
					
					currentState = TX_RX;
				}
				break;
			}
			
			case TX_RX: {
				GNSS_get_data();
				LoRa_Transmit();
				dataRecieved = LoRa_Recieve();
				
				if (HAL_GetTick() - refreshedTime > SCREEN_REFRESH_PERIOD) {
					refreshedTime = HAL_GetTick();
					currentState = DRAW_SATS;
				}
				break;
			}
			
			case DRAW_SATS: {
				
				char string_buffer[50];
				sprintf(string_buffer, "Sats Visible: %d", currentData.satellites);
				ILI9486_FillRect(10, 10, 200, 20, BLACK); // ERASE 
				ILI9486_DrawString(10, 10, string_buffer, Font_11x18, RED, BLACK); // DRAW
				
				sprintf(message, "\n DRAW - Sats visible: %d\r\n", currentData.satellites);
				print_msg(message);
				
				if (dataRecieved) {
					currentState = DRAW_RSSI;
				} else {
					currentState = DRAW_FAILED;
				}
				break;
			}
			
			case DRAW_FAILED: {
				ILI9486_FillRect(200, 10, 200, 20, BLACK); // ERASE 
				ILI9486_DrawString(200, 10, "Recieve Failed", Font_11x18, RED, BLACK);
				
				sprintf(message, "\nRecieved Failed\r\n");
				print_msg(message);
				
				currentState = TX_RX;
				break;
			}
			
			case DRAW_RSSI: {
				sprintf(message, "RSSI: something \r\n" ); // TODO
				print_msg(message);
				
				if (currentData.satellites >= 3) { // AND THE OTHER ALSO SAT >= 3
					currentState = PLOT;
				} else {
					currentState = TX_RX;
				}
				break;
			}
			
			case PLOT: {
				calculate_position();
				draw_position();
				
				currentState = TX_RX;
			}
			break;
		}
		Log_System_Status(currentState, currentData.satellites);
		HAL_Delay(1);
			
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, TFT_RS_Pin|TFT_D1_Pin|TFT_D6_Pin|TFT_D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TFT_WR_GPIO_Port, TFT_WR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LoRa_NSS_GPIO_Port, LoRa_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TFT_RD_Pin|TFT_RST_Pin|LoRa_RESET_Pin|TFT_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, TFT_D0_Pin|TFT_D7_Pin|TFT_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, USB_PowerSwitchOn_Pin|TFT_D2_Pin|TFT_D3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_RS_Pin TFT_D1_Pin TFT_D6_Pin TFT_D4_Pin */
  GPIO_InitStruct.Pin = TFT_RS_Pin|TFT_D1_Pin|TFT_D6_Pin|TFT_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : TFT_WR_Pin */
  GPIO_InitStruct.Pin = TFT_WR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(TFT_WR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LoRa_NSS_Pin */
  GPIO_InitStruct.Pin = LoRa_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LoRa_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LoRa_RESET_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LoRa_RESET_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_RD_Pin TFT_RST_Pin TFT_CS_Pin */
  GPIO_InitStruct.Pin = TFT_RD_Pin|TFT_RST_Pin|TFT_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_D0_Pin TFT_D7_Pin TFT_D5_Pin */
  GPIO_InitStruct.Pin = TFT_D0_Pin|TFT_D7_Pin|TFT_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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

  /*Configure GPIO pins : TFT_D2_Pin TFT_D3_Pin */
  GPIO_InitStruct.Pin = TFT_D2_Pin|TFT_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t sd3031_readReg(uint8_t reg, void* pBuf, size_t size) {
    // The SD3031 address is 0x32 (standard) or adjusted by your library defines
    // HAL expects address shifted left by 1: (0x32 << 1)
    if (HAL_I2C_Mem_Read(&hi2c2, (0x32 << 1), reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)pBuf, size, 100) == HAL_OK) {
        return 0;
    }
    return 1;
}

void sd3031_writeReg(uint8_t reg, void* pBuf, size_t size) {
    HAL_I2C_Mem_Write(&hi2c2, (0x32 << 1), reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)pBuf, size, 100);
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
#ifdef USE_FULL_ASSERT
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
