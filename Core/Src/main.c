/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#define M24C04

#ifdef M24C04
  #define I2C_DEVICE_ADDR (0xA0) /*should be changed depends : 1  0  1  0 E2 E1 A8 RW */
#else
  #define I2C_DEVICE_ADDR (0xA8) /*should be changed depends : 1  0  1  0  1  0  0 RW */
#endif

static int8_t i2c_write_eeprom(uint16_t address, uint8_t *buffer, uint16_t size);
static int8_t i2c_read_eeprom(uint16_t address, uint8_t *buffer, uint16_t size);

#define MAX_LEN (256)
uint8_t rx_buff[MAX_LEN];
uint8_t tx_buff[MAX_LEN] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
    0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,
    0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F,
    0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F,
    0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F,
    0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F,
    0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F,
    0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF,
    0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF,
    0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF,
    0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF,
    0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF,
    0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF,
};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  memset(rx_buff, 0x00, MAX_LEN);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
  */
  LL_PWR_DisableUCPDDeadBattery();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  i2c_write_eeprom(0x0000, tx_buff, MAX_LEN);
  i2c_read_eeprom(0x0000, rx_buff, MAX_LEN);

  while (1)
  {
    LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5);
    LL_mDelay(100);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSI_Enable();
   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 85, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1�s transition state at intermediate medium speed clock*/
  for (__IO uint32_t i = (170 >> 1); i !=0; i--);

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(170000000);

  LL_SetSystemCoreClock(170000000);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
#ifdef M24C04
  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**I2C1 GPIO Configuration
  PA13   ------> I2C1_SCL
  PA14   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* USER CODE BEGIN I2C1_Init 1 */
#else
  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB8-BOOT0   ------> I2C1_SCL
  PB9   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
#endif
  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x10802D9B;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);

  /**/
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void i2c_soft_reset(void)
{
  I2C1->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN));
  I2C1->CR1 &= ~I2C_CR1_PE;

  while(I2C1->CR1 & I2C_CR1_PE){};
  I2C1->CR1 |= I2C_CR1_PE;
}

static int8_t i2c_wait_flag(uint32_t flag, uint32_t status, int32_t timeout)
{
  while(READ_BIT(I2C1->ISR, flag) == status)
  {
    if(timeout == 0)
    {
      i2c_soft_reset();
      return -1;
    }

    timeout--;
  }
  return 0;
}

static int8_t i2c_write_eeprom(uint16_t address, uint8_t *buffer, uint16_t size)
{
  uint16_t XferCount;
  XferCount = size;

  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);  //WC Control

  do
  {
    //Busy check
    if(i2c_wait_flag(I2C_ISR_BUSY, I2C_ISR_BUSY, 0x00FFFFFF) != 0)
    {
      LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);  //WC Control
      return -1;
    }

    //Set configuration and start condition
    LL_I2C_HandleTransfer(I2C1,
#ifdef M24C04
                          I2C_DEVICE_ADDR | ((uint16_t)(address >> 7) & (0x0002)),
#else
                          I2C_DEVICE_ADDR,
#endif
                          LL_I2C_ADDRSLAVE_7BIT,
                          2,
                          LL_I2C_MODE_AUTOEND,
                          LL_I2C_GENERATE_START_WRITE);

    if(i2c_wait_flag(I2C_ISR_TXIS, 0, 0x00FFFFFF) != 0)
    {
      LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);  //WC Control
      return -1;
    }

    //Set memory address
    LL_I2C_TransmitData8(I2C1, ((uint8_t)((uint16_t)((address) & (uint16_t)(0x00FFU)))));

    //transfer data
    if(i2c_wait_flag(I2C_ISR_TXIS, 0, 0x00FFFFFF) != 0)
    {
      LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);  //WC Control
      return -1;
    }

    LL_I2C_TransmitData8(I2C1, *buffer);

    //wait for stop condition
    if(i2c_wait_flag(I2C_ISR_STOPF, 0, 0x00FFFFFF) != 0)
    {
      LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);  //WC Control
      return -1;
    }

    //clear stop flag
    LL_I2C_ClearFlag_STOP(I2C1);

    //reset CR2 of I2C
    I2C1->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN));

    XferCount--;
    buffer++;
    address++;

    LL_mDelay(5); /* Write time Max 5msec */

  }while(XferCount > 0);

  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);  //WC Control
  return 0;
}


static int8_t i2c_read_eeprom(uint16_t address, uint8_t *buffer, uint16_t size)
{
  uint16_t XferCount, XferSize;

  XferCount = size;

  //Busy check
  if(i2c_wait_flag(I2C_ISR_BUSY, I2C_ISR_BUSY, 0x00FFFFFF) != 0)
  {
    return -1;
  }

  //Set configuration and start condition
  LL_I2C_HandleTransfer(I2C1,
#ifdef M24C04
                        I2C_DEVICE_ADDR | ((uint16_t)(address >> 7) & (0x0002)),
#else
                        I2C_DEVICE_ADDR,
#endif
                        LL_I2C_ADDRSLAVE_7BIT,
                        1,
                        LL_I2C_MODE_SOFTEND,
                        LL_I2C_GENERATE_START_WRITE);

  if(i2c_wait_flag(I2C_ISR_TXIS, 0, 0x00FFFFFF) != 0)
  {
    return -1;
  }

  //Set memory address
  LL_I2C_TransmitData8(I2C1, ((uint8_t)((uint16_t)((address) & (uint16_t)(0x00FFU)))));

  if(i2c_wait_flag(I2C_ISR_TC, 0, 0x00FFFFFF) != 0)
  {
    return -1;
  }


  //Set device configuration and start condition again
  if(XferCount > 255)
  {
    XferSize = 255;
    LL_I2C_HandleTransfer(I2C1,
#ifdef M24C04
                          I2C_DEVICE_ADDR | ((uint16_t)(address >> 7) & (0x0002)),
#else
                          I2C_DEVICE_ADDR,
#endif
                          LL_I2C_ADDRSLAVE_7BIT,
                          XferSize,
                          LL_I2C_MODE_RELOAD,
                          LL_I2C_GENERATE_START_READ);
  }
  else
  {
    XferSize = XferCount;
    LL_I2C_HandleTransfer(I2C1,
#ifdef M24C04
                          I2C_DEVICE_ADDR | ((uint16_t)(address >> 7) & (0x0002)),
#else
                          I2C_DEVICE_ADDR,
#endif
                          LL_I2C_ADDRSLAVE_7BIT,
                          XferSize,
                          LL_I2C_MODE_AUTOEND,
                          LL_I2C_GENERATE_START_READ);
  }

  //read data
  do
  {
    //Check RX empty
    if(i2c_wait_flag(I2C_ISR_RXNE, 0, 0x00FFFFFF) != 0)
    {
      return -1;
    }

    *buffer = LL_I2C_ReceiveData8(I2C1);
    buffer++;
    XferSize--;
    XferCount--;

    //Check and re-configuration for N-byte
    if ((XferCount != 0U) && (XferSize == 0U))
    {
      if(i2c_wait_flag(I2C_ISR_TCR, 0, 0x00FFFFFF) != 0)
      {
        return -1;
      }

      if (XferCount > 255)
      {
        XferSize = 255;
        LL_I2C_HandleTransfer(I2C1,
#ifdef M24C04
                              I2C_DEVICE_ADDR | ((uint16_t)(address >> 7) & (0x0002)),
#else
                              I2C_DEVICE_ADDR,
#endif
                              LL_I2C_ADDRSLAVE_7BIT,
                              XferSize,
                              LL_I2C_MODE_RELOAD,
                              LL_I2C_GENERATE_NOSTARTSTOP);
      }
      else
      {
        XferSize = XferCount;
        LL_I2C_HandleTransfer(I2C1,
#ifdef M24C04
                              I2C_DEVICE_ADDR | ((uint16_t)(address >> 7) & (0x0002)),
#else
                              I2C_DEVICE_ADDR,
#endif
                              LL_I2C_ADDRSLAVE_7BIT,
                              XferSize,
                              LL_I2C_MODE_AUTOEND,
                              LL_I2C_GENERATE_NOSTARTSTOP);
      }
    }
  }while(XferCount > 0);

  //wait for stop condition
  if(i2c_wait_flag(I2C_ISR_STOPF, 0, 0x00FFFFFF) != 0)
  {
    return -1;
  }

  //clear stop flag
  LL_I2C_ClearFlag_STOP(I2C1);

  //reset CR2 of I2C
  I2C1->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN));

  return 0;
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
