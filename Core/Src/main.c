/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vpc3.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
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

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/*******************************************************************************************
	VAR I2C
*******************************************************************************************/
#define I2C_SLAVE_ADDR  (0x10 << 1)
#define RX_BUFFER_SIZE  60
#define TX_BUFFER_SIZE  2
uint8_t i2c_rx_buffer[RX_BUFFER_SIZE];
uint8_t i2c_tx_buffer[TX_BUFFER_SIZE] = {0x0, 0x0}; // Datos a enviar al maestro
uint8_t Impresion_1 = 0;
uint8_t Impresion_2 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Cuando el maestro quiere escribir
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  // Aquí los datos ya están en i2c_rx_buffer
}

// Cuando el maestro quiere leer
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  // Se completó la transmisión de datos
}

// Cuando se detecta la dirección del esclavo (¡IMPORTANTE!)
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  if (hi2c->Instance == I2C1)
  {
      if (TransferDirection == I2C_DIRECTION_TRANSMIT)  // Maestro escribirá
      {
          HAL_I2C_Slave_Seq_Receive_IT(hi2c, i2c_rx_buffer, sizeof(i2c_rx_buffer), I2C_FIRST_AND_LAST_FRAME);
      }
      else  // Maestro leerá
      {
          HAL_I2C_Slave_Seq_Transmit_IT(hi2c, i2c_tx_buffer, sizeof(i2c_tx_buffer), I2C_FIRST_AND_LAST_FRAME);
      }
  }
}

// Reinicia la escucha tras cada sesión
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  HAL_I2C_EnableListen_IT(hi2c);  // Importante volver a escuchar
}

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
  // Habilitar DWT CYCCNT para DelayUs preciso (no interfiere con SysTick/HAL_Delay)
  // Si ya estuviera habilitado, estas líneas no causan efecto adverso.
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // ============================================================================
  // DIAGNÓSTICO DE CLOCK - Verificar que el sistema esté corriendo a 84MHz
  // Si ves caracteres basura, el clock no está configurado correctamente
  // ============================================================================
  {
    char diag_buf[80];
    int n;

    // Pequeña espera para estabilizar el UART
    for(volatile int i = 0; i < 100000; i++);

    // Mensaje de inicio dividido en partes para evitar overflow del buffer
    n = snprintf(diag_buf, sizeof(diag_buf), "\r\n\r\n========================================\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_buf, n, 100);
    
    n = snprintf(diag_buf, sizeof(diag_buf), "[BOOT] STM32F401CC VPC3+ Profibus Slave\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_buf, n, 100);
    
    n = snprintf(diag_buf, sizeof(diag_buf), "========================================\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_buf, n, 100);

    n = snprintf(diag_buf, sizeof(diag_buf),
        "[CLK] SystemCoreClock = %lu Hz\r\n", SystemCoreClock);
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_buf, n, 100);

    // Verificar que el clock sea ~84MHz
    if(SystemCoreClock < 80000000 || SystemCoreClock > 88000000) {
      n = snprintf(diag_buf, sizeof(diag_buf),
          "[ERROR] Clock incorrecto! Esperado: 84MHz\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)diag_buf, n, 100);
    } else {
      n = snprintf(diag_buf, sizeof(diag_buf),
          "[OK] Clock configurado correctamente\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)diag_buf, n, 100);
    }

    // Mostrar fuente del clock
    uint32_t clk_src = __HAL_RCC_GET_SYSCLK_SOURCE();
    const char *src_name = "UNKNOWN";
    if(clk_src == RCC_SYSCLKSOURCE_STATUS_HSI) src_name = "HSI";
    else if(clk_src == RCC_SYSCLKSOURCE_STATUS_HSE) src_name = "HSE";
    else if(clk_src == RCC_SYSCLKSOURCE_STATUS_PLLCLK) src_name = "PLL";

    n = snprintf(diag_buf, sizeof(diag_buf),
        "[CLK] Fuente SYSCLK: %s\r\n", src_name);
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_buf, n, 100);

    // Mostrar APB1 y APB2 clocks
    n = snprintf(diag_buf, sizeof(diag_buf),
        "[CLK] APB1=%lu Hz, APB2=%lu Hz\r\n",
        HAL_RCC_GetPCLK1Freq(), HAL_RCC_GetPCLK2Freq());
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_buf, n, 100);

    n = snprintf(diag_buf, sizeof(diag_buf),
        "========================================\r\n\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)diag_buf, n, 100);
  }

  // --- Inicialización VPC3 ---
  HAL_Delay(300);
  DpAppl_SetResetVPC3Channel1();
  HAL_Delay(100); // Esperar que VPC3 se resetee completamente

  {
    char msg[64];
    int n = snprintf(msg, sizeof(msg), "[VPC3] Iniciando inicializacion...\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, n, 100);
  }

  DpAppl_ProfibusInit();
  HAL_Delay(200); // Esperar que Profibus se inicialice completamente

  // --- Inicializar datos de Profibus con valores por defecto ---
  // Inicializar datos que el F411 enviará al PLC (pueden ser diferentes de cero)
  sDpAppl.abDpInputData[0] = 0xAA;  // Heartbeat inicial
  sDpAppl.abDpInputData[1] = 0x55;  // Pattern para verificar comunicación
  sDpAppl.abDpInputData[2] = 0x01;  // Status: Inicializado

  // --- Activar escucha I2C ---
  HAL_I2C_EnableListen_IT(&hi2c1);  // �? CR�?TICO: Sin esta línea, I2C no escucha

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // --- Lógica principal de Profibus (portado del main.c original) ---
	  DpAppl_ProfibusMain();

	  // Log de cambios en datos de Profibus (SIEMPRE mostrar cambios, pero con throttling)
	  static uint8_t prev_outputs[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	  static uint32_t last_log_time = 0;
	  bool changed = false;
	  
	  for(int i = 0; i < 8; i++) {
	      if(sDpAppl.abDpOutputData[i] != prev_outputs[i]) {
	          changed = true;
	          prev_outputs[i] = sDpAppl.abDpOutputData[i];
	      }
	  }
	  
	  // LOG: Mostrar cambios
	  uint32_t current_time = HAL_GetTick();
	  if (changed && (current_time - last_log_time >= 100)) {
	      __disable_irq();
	      char buf[256];
	      int n = snprintf(buf, sizeof(buf),
	          "[PROFIBUS] PLC->F411: [%02X %02X %02X %02X %02X %02X %02X %02X] | F411->PLC: [%02X %02X %02X %02X %02X %02X %02X %02X]\r\n",
	          sDpAppl.abDpOutputData[0], sDpAppl.abDpOutputData[1], sDpAppl.abDpOutputData[2], sDpAppl.abDpOutputData[3],
	          sDpAppl.abDpOutputData[4], sDpAppl.abDpOutputData[5], sDpAppl.abDpOutputData[6], sDpAppl.abDpOutputData[7],
	          sDpAppl.abDpInputData[0], sDpAppl.abDpInputData[1], sDpAppl.abDpInputData[2], sDpAppl.abDpInputData[3],
	          sDpAppl.abDpInputData[4], sDpAppl.abDpInputData[5], sDpAppl.abDpInputData[6], sDpAppl.abDpInputData[7]);

	  HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, 100);
	  __enable_irq();
	  last_log_time = current_time;
	  }

	  // --- Procesamiento de datos I2C (opcional) ---
	  // Los datos I2C ya están en i2c_rx_buffer[] gracias a los callbacks
	  // Aquí puedes leer y procesar los datos recibidos desde el maestro (C_2.0.7)
	  
	  // Ejemplo: leer datos específicos y mostrarlos cada cierto tiempo
	  static uint32_t last_i2c_log = 0;
	  if (current_time - last_i2c_log >= 1000) { // Cada 1 segundo
	      Impresion_1 = i2c_rx_buffer[3];
	      Impresion_2 = i2c_rx_buffer[4];
	      i2c_tx_buffer[1] = sDpAppl.abDpOutputData[0]; // Actualizar dato a enviar al maestro
	      
	      __disable_irq();
	      char i2c_buf[128];
	      int i2c_n = snprintf(i2c_buf, sizeof(i2c_buf),
	          "[I2C] RX[3]=0x%02X (%d), RX[4]=0x%02X (%d) | TX[1]=%d\r\n",
	          i2c_rx_buffer[3], i2c_rx_buffer[3],
	          i2c_rx_buffer[4], i2c_rx_buffer[4],
	          i2c_tx_buffer[1]);
	      HAL_UART_Transmit(&huart2, (uint8_t*)i2c_buf, i2c_n, 100);
	      __enable_irq();
	      last_i2c_log = current_time;
	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  hi2c1.Init.OwnAddress1 = 32;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  huart2.Instance = USART1;
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VPC3_CS_GPIO_Port, VPC3_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VPC3_RESET_GPIO_Port, VPC3_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : VPC3_INT_Pin */
  GPIO_InitStruct.Pin = VPC3_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VPC3_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VPC3_CS_Pin */
  GPIO_InitStruct.Pin = VPC3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VPC3_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VPC3_RESET_Pin */
  GPIO_InitStruct.Pin = VPC3_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VPC3_RESET_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
