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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
#include "lcd.h"
#include "keypad.h"
#include "servo.h"
#include "RC522.h"
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

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void handle_key_input(char key, uint8_t* door_open, uint8_t* cursor_index);
int changePassword(void);
void show_mode_selection(void);
void handle_rfid_mode(uint8_t* door_open);
void handle_password_mode(uint8_t* door_open);
uint8_t read_rfid_card(uint8_t* card_data);
uint8_t compare_rfid_card(uint8_t* card_data);
void print_card_data(uint8_t* card_data); // 디버깅용 함수 추가
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char password[5] = "1234";
char entered[5];
uint8_t index_ = 0;
uint8_t system_mode = 0; // 0: 모드 선택, 1: RFID 모드, 2: 비밀번호 모드
uint8_t authorized_card[4] = {0, 0, 0, 0}; // 초기에는 빈 카드, 첫 번째 카드로 등록
uint8_t card_registered = 0; // 카드 등록 여부
uint8_t rfid_card_present = 0; // 이전 카드 상태
uint32_t last_rfid_check = 0;
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
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  servo_init();
  lcd_init(&hi2c1);

  // MFRC522 초기화
  MFRC522_Init();

  show_mode_selection();

    index_ = 0;
    memset(entered, 0, sizeof(entered));
    uint8_t i = 0;
    uint8_t door_open = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (system_mode == 0) {
	        // 모드 선택 대기
	        char key = keypad_get_key();
	        if (key == '1') {
	          system_mode = 1; // RFID 모드
	          lcd_clear();
	          lcd_set_cursor(0, 0);
	          if (!card_registered) {
	            lcd_send_string("First: Register");
	            lcd_set_cursor(1, 0);
	            lcd_send_string("Present Card...");
	          } else {
	            lcd_send_string("RFID Mode");
	            lcd_set_cursor(1, 0);
	            lcd_send_string("Present Card...");
	          }
	          HAL_Delay(1000);
	        }
	        else if (key == '2') {
	          system_mode = 2; // 비밀번호 모드
	          lcd_clear();
	          lcd_set_cursor(0, 0);
	          lcd_send_string("Password Mode");
	          HAL_Delay(500);
	          lcd_clear();
	          lcd_set_cursor(0, 0);
	          lcd_send_string("Enter Password:");
	          index_ = 0;
	          i = 0;
	          memset(entered, 0, sizeof(entered));
	        }
	      }
	      else if (system_mode == 1) {
	        // RFID 모드
	        handle_rfid_mode(&door_open);
	      }
	      else if (system_mode == 2) {
	        // 비밀번호 모드
	        char key = keypad_get_key();
	        if (key != 0) {
	          handle_key_input(key, &door_open, &i);
	        }
	      }

	      // 모드 변경 (B키로 모드 선택으로 돌아가기, 문이 잠겨있을 때만)
	      if (!door_open && system_mode != 0) {
	        char key = keypad_get_key();
	        if (key == 'B') {
	          system_mode = 0;
	          show_mode_selection();
	          index_ = 0;
	          i = 0;
	          memset(entered, 0, sizeof(entered));
	        }
	      }

	      HAL_Delay(10); // CPU 부하 감소
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  HAL_GPIO_WritePin(GPIOA, R1_Pin|R2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, R3_Pin|R4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R1_Pin R2_Pin */
  GPIO_InitStruct.Pin = R1_Pin|R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : C1_Pin C2_Pin C3_Pin C4_Pin */
  GPIO_InitStruct.Pin = C1_Pin|C2_Pin|C3_Pin|C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : R3_Pin R4_Pin */
  GPIO_InitStruct.Pin = R3_Pin|R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_Pin SDA_Pin */
  GPIO_InitStruct.Pin = RST_Pin|SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void show_mode_selection(void)
{
  lcd_clear();
  lcd_set_cursor(0, 0);
  lcd_send_string("Select Mode:");
  lcd_set_cursor(1, 0);
  lcd_send_string("1=RFID 2=Pass");
}

void handle_rfid_mode(uint8_t* door_open)
{
  uint32_t current_time = HAL_GetTick();

  // 100ms마다 RFID 체크
  if (current_time - last_rfid_check > 100) {
    last_rfid_check = current_time;

    uint8_t card_data[4];
    uint8_t card_present = read_rfid_card(card_data);

    // 카드 상태 변화 감지 (새로 감지되었을 때만 처리)
    if (card_present && !rfid_card_present) {

      printf("Card detected: ");
      print_card_data(card_data);

      if (!card_registered) {
        // 첫 번째 카드 등록
        for (int i = 0; i < 4; i++) {
          authorized_card[i] = card_data[i];
        }
        card_registered = 1;

        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_send_string("Card Registered!");
        lcd_set_cursor(1, 0);
        lcd_send_string("Access Granted");
        HAL_Delay(2000);

        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_send_string("RFID Mode");
        lcd_set_cursor(1, 0);
        lcd_send_string("Present Card...");
      }
      else if (compare_rfid_card(card_data)) {
        // 등록된 카드와 일치
        servo_unlock();
        *door_open = 1;

        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_send_string("   Door Open     ");
        lcd_set_cursor(1, 0);
        lcd_send_string("Press # to Lock");

      }
      else {
        // 승인되지 않은 카드
        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_send_string(" Access Denied  ");
        lcd_set_cursor(1, 0);
        lcd_send_string("Invalid Card");
        HAL_Delay(1500);

        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_send_string("RFID Mode");
        lcd_set_cursor(1, 0);
        lcd_send_string("Present Card...");
      }
    }

    rfid_card_present = card_present;
  }

  // 문이 열려있을 때 #키로 잠금
  if (*door_open) {
    char key = keypad_get_key();
    if (key == '#') {
      servo_lock();
      *door_open = 0;

      lcd_clear();
      lcd_set_cursor(0, 0);
      lcd_send_string("   Door Locked   ");
      HAL_Delay(1000);

      lcd_clear();
      lcd_set_cursor(0, 0);
      lcd_send_string("RFID Mode");
      lcd_set_cursor(1, 0);
      lcd_send_string("Present Card...");
    }
  }
}

// 개선된 RFID 카드 읽기 함수
uint8_t read_rfid_card(uint8_t* card_data)
{
  uint8_t status;
  uint8_t str[5];

  // 1단계: 카드 감지
  status = MFRC522_Request(PICC_REQIDL, str);
  if (status != MI_OK) {
    return 0; // 카드 없음
  }

  // 2단계: UID 읽기
  status = MFRC522_Anticoll(str);
  if (status != MI_OK) {
    return 0; // UID 읽기 실패
  }

  // UID 복사 (4바이트만)
  for (int i = 0; i < 4; i++) {
    card_data[i] = str[i];
  }

  return 1; // 성공
}

uint8_t compare_rfid_card(uint8_t* card_data)
{
  for (int i = 0; i < 4; i++) {
    if (card_data[i] != authorized_card[i]) {
      return 0; // 카드가 일치하지 않음
    }
  }
  return 1; // 카드가 일치함
}

// 디버깅용 카드 데이터 출력 함수
void print_card_data(uint8_t* card_data)
{
  printf("%02X %02X %02X %02X\r\n",
         card_data[0], card_data[1], card_data[2], card_data[3]);
}
int changePassword(void){
	char newPass[5];

	lcd_clear();
	lcd_set_cursor(0,0);
	lcd_send_string("Change Password?");
	lcd_set_cursor(1,0);
	lcd_send_string("A=Yes, B=No:");

	char choice = 0;
	while(choice != 'A' && choice != 'B') {
		choice = keypad_get_key();
		HAL_Delay(10);
	}

	if(choice == 'B'){
		lcd_clear();
		lcd_send_string("Canceled");
		HAL_Delay(1000);
		return 0;
	}

	lcd_clear();
	lcd_set_cursor(0,0);
	lcd_send_string("New Password:");

	for(int i = 0; i < 4; i++){
		char k = 0;
		while((k = keypad_get_key()) == 0){
			HAL_Delay(10); // 짧은 딜레이만 유지 (키패드 안정성을 위해)
		}

		if(k >= '0' && k <= '9'){
			newPass[i] = k;
			lcd_set_cursor(1,i);
			lcd_send_data('*');
		} else {
			i--;
			continue;
		}

		// 키 입력 후 딜레이 제거 - 바로 다음 입력 받기
	}

	newPass[4] = '\0';
	if(strcmp(newPass, password) != 0) {
		for(int i = 0; i < 4; i++){
			password[i] = newPass[i];
		}

		lcd_clear();
		lcd_set_cursor(0,0);
		lcd_send_string("Password Updated");
		HAL_Delay(1500);
		return 1;
	} else {
		lcd_clear();
		lcd_set_cursor(0,0);
		lcd_send_string("Same Password!");
		HAL_Delay(1500);
		return 0;
	}
}

void handle_key_input(char key, uint8_t* door_open, uint8_t* cursor_index)
{
  if (*door_open)
  {
    if (key == '#')
    {
      servo_lock();
      *door_open = 0;

      lcd_clear();
      lcd_set_cursor(0, 0);
      lcd_send_string("   Door Locked   ");
      HAL_Delay(1000);

      lcd_clear();
      lcd_set_cursor(0, 0);
      lcd_send_string("Enter Password:");

      index_ = 0;
      *cursor_index = 0;
      memset(entered, 0, sizeof(entered));
    }
    return;
  }

  if (key == 'C')
  {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_send_string("Enter Password:");
    index_ = 0;
    *cursor_index = 0;
    memset(entered, 0, sizeof(entered));
  }
  else if (key >= '0' && key <= '9')
  {
    if (index_ < 4)
    {
      lcd_set_cursor(1, (*cursor_index));
      lcd_send_data('*');
      entered[index_++] = key;
      (*cursor_index)++;
      // 딜레이 제거 - 빠른 입력 가능
    }
  }
  else if (key == 'D')
  {
    if (index_ == 4)
    {
      if (strcmp(entered, password) == 0)
      {
        servo_unlock();
        *door_open = 1;

        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_send_string("   Door Open     ");
        lcd_set_cursor(1, 0);
        lcd_send_string("Press # to Lock");
      }
      else
      {
        lcd_set_cursor(0, 0);
        lcd_send_string(" Access Denied  ");
        lcd_set_cursor(1, 0);
        lcd_send_string("                ");
        HAL_Delay(1000);

        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_send_string("Enter Password:");
      }

      index_ = 0;
      *cursor_index = 0;
      memset(entered, 0, sizeof(entered));
    }
  }
  else if (key == 'A')
  {
    changePassword();

    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_send_string("Enter Password:");

    index_ = 0;
    *cursor_index = 0;
    memset(entered, 0, sizeof(entered));
  }
}

#ifdef __GNUC__
/* GCC 컴파일러에서 printf는 이 함수를 호출합니다 */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
  /* USART2로 한 문자를 보내고 전송이 끝날 때까지 대기합니다 */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
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
