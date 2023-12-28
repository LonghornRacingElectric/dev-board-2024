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
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "VcuModel.h"
#include "analog.h"
#include "inv.h"
#include "pdu.h"
#include "faults.h"
#include "angel_can.h"
#include <cmath>
#include <vector>
#include <string>
using namespace std;
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
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
VcuModel vcuModel;
VcuParameters vcuParameters;
VcuInput vcuInput;
VcuOutput vcuOutput;

void ext_red_led(GPIO_PinState on) {
    HAL_GPIO_WritePin(ext_red_led_GPIO_Port, ext_red_led_Pin, on);
}
void ext_yellow_led(GPIO_PinState on) {
    HAL_GPIO_WritePin(ext_yellow_led_GPIO_Port, ext_yellow_led_Pin, on);
}
void ext_green_led(GPIO_PinState on) {
    HAL_GPIO_WritePin(ext_green_led_GPIO_Port, ext_green_led_Pin, on);
}
void ext_blue_led(GPIO_PinState on) {
    HAL_GPIO_WritePin(ext_blue_led_GPIO_Port, ext_blue_led_Pin, on);
}
void ext_white_led(GPIO_PinState on) {
    HAL_GPIO_WritePin(ext_white_led_GPIO_Port, ext_white_led_Pin, on);
}
void ext_rgb_led(GPIO_PinState red, GPIO_PinState green, GPIO_PinState blue) {
    HAL_GPIO_WritePin(ext_rgb_red_GPIO_Port, ext_rgb_red_Pin, red);
    HAL_GPIO_WritePin(ext_rgb_green_GPIO_Port, ext_rgb_green_Pin, green);
    HAL_GPIO_WritePin(ext_rgb_blue_GPIO_Port, ext_rgb_blue_Pin, blue);
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_FDCAN2_Init();
  MX_LPUART1_UART_Init();
  MX_UART7_Init();
  // MX_SDMMC1_SD_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

//  volatile uint32_t last_time_recorded = 0;
//  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
//  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
//  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(SysTick_IRQn);
//
//  BSPD bspd = {0, 0, 0, 0, 0};
//

  vcuParameters.appsLowPassFilterTimeConstant = 0.000f;
  vcuParameters.appsImplausibilityTime = 0.100f;
  vcuParameters.appsPlausibilityRange = 0.10f;
  vcuParameters.apps1VoltageMin = 0.0f;
  vcuParameters.apps1VoltageMax = 3.3f;
  vcuParameters.apps2VoltageMin = 0.0f;
  vcuParameters.apps2VoltageMax = 3.3f;
  vcuParameters.appsDeadZonePct = 0.05f;

  vcuParameters.bseLowPassFilterTimeConstant = 0.00f;
  vcuParameters.bseImplausibilityTime = 0.100f;
  vcuParameters.bseVoltageMin = 0.0f;
  vcuParameters.bseVoltageMax = 3.3f;
  vcuParameters.bsePressureMin = 0.0f;
  vcuParameters.bsePressureMax = 1000.0f;

  vcuParameters.stomppMechanicalBrakesThreshold = 100.0f;
  vcuParameters.stomppAppsCutoffThreshold = 0.25f;
  vcuParameters.stomppAppsRecoveryThreshold = 0.05f;

  vcuParameters.mapPedalToTorqueRequest = CurveParameter(1.0f, 230.0f);
  vcuParameters.mapDerateMotorTemp = CurveParameter();
  vcuParameters.mapDerateInverterTemp = CurveParameter();
  vcuParameters.mapDerateBatteryTemp = CurveParameter();
  vcuParameters.mapDerateBatterySoc = CurveParameter();

  vcuParameters.prndlBrakeToStartThreshold = 100.0f;
  vcuParameters.prndlBuzzerDuration = 2.0f;
  vcuParameters.prndlSwitchDebounceDuration = 0.100f;

  vcuModel.setParameters(&vcuParameters);

  // left out sus sensors for now
  vcu_fault_vector = 0;
    if(adc_start(&hadc1) != 0){
        FAULT_SET(&vcu_fault_vector, FAULT_VCU_ADC);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
        adc_stop(&hadc1);
    }
    AnalogVoltages adcVals;

  if(can_init(&hfdcan2) != HAL_OK){
      vcu_fault_vector |= FAULT_VCU_CAN;
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
  }
  inverter_init();
  pdu_init();

  HAL_GPIO_WritePin(CAN_TERM_GPIO_Port, CAN_TERM_Pin, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  /* This will write to update CAN Inverter to 1 Mbit/s */
    inverter_updateCANBitRate(1000);

  /* This will write to update Torque Limit to whatever value (in this case 120 Nm) */
    inverter_setTorqueLimit(120);

  /* This will clear any inverter faults */
  inverter_resetFaults();

  /* This will selectively disable any broadcasts from the inverter */
    inverter_enableFaults(0xFFFF3CE5);

  InverterStatus invStatus;
  PDUStatus pduStatus;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    adc_get(&adcVals);
    vcuInput.driveSwitch = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
    vcuInput.inverterReady = true;
    vcuInput.apps1 = adcVals.apps1;
    vcuInput.apps2 = adcVals.apps2;
    vcuInput.bse1 = adcVals.bse1;
    vcuInput.bse2 = adcVals.bse2;
    vcuInput.steeringWheelPotVoltage = adcVals.steer;

    vcuModel.evaluate(&vcuInput, &vcuOutput, 0.003f);

    htim3.Instance->CCR3 = (uint16_t) (vcuOutput.inverterTorqueRequest / 230.0f * 65535.0f);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, (GPIO_PinState) !vcuOutput.prndlState);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, (GPIO_PinState) (vcuOutput.faultApps || vcuOutput.faultBse || vcuOutput.faultStompp));

    HAL_GPIO_WritePin(OUT_buzzer_GPIO_Port, OUT_buzzer_Pin, (GPIO_PinState) vcuOutput.r2dBuzzer);
    if(vcuInput.bse1 > 0.5f || vcuInput.bse2 > 0.5f){
        ext_white_led(GPIO_PIN_SET);
    } else {
        ext_white_led(GPIO_PIN_RESET);
    }
    unsigned int CAN_error = inverter_sendTorqueCommand(vcuOutput.inverterTorqueRequest, 0, vcuInput.inverterReady, 3);
    if(CAN_error > 0){
      vcu_fault_vector |= FAULT_VCU_INV;
    }

    CAN_error += can_processRxFifo();
    if(CAN_error > 0){
      vcu_fault_vector |= FAULT_VCU_CAN;
        ext_red_led(GPIO_PIN_SET);
    }
    else{
        ext_red_led(GPIO_PIN_RESET);
    }
    inverter_update(&invStatus);
    if(abs(invStatus.torqueCommand - vcuOutput.inverterTorqueRequest) < 0.1f){
        ext_green_led(GPIO_PIN_SET);
    }
    else{
        ext_green_led(GPIO_PIN_RESET);
    }
  uint32_t pdu_error = pdu_update(&pduStatus, &vcuOutput, 0.003f);
    if(FAULT_CHECK(&vcu_fault_vector, FAULT_VCU_PDU)){
        ext_red_led(GPIO_PIN_SET);
    }
    else{
        ext_red_led(GPIO_PIN_RESET);
    }
    if(pdu_error == 2){
        ext_rgb_led(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
    }
    else{
        ext_rgb_led(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);
    }

    vcu_fault_vector = 0;
    can_clearMailboxes();

    HAL_Delay(3);

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

  /*AXI clock gating */
  RCC->CKGAENR = 0xFFFFFFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 16;
  PeriphClkInitStruct.PLL2.PLL2P = 3;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
    while (1) {
      //Write to Red LED
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
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
