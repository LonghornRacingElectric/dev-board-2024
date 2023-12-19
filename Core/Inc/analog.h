//
// Created by yashk on 10/29/2023.
//

#ifndef VCU_FIRMWARE_2024_ANALOG_H
#define VCU_FIRMWARE_2024_ANALOG_H

#include "stm32h7xx_hal.h"
#include "VcuModel.h"
#define NUM_SENSORS 5
#define NUM_VALUES_PER_SENSOR 32
#define APPS1 3
#define APPS2 0
#define BSE1 2
#define BSE2 1
#define STEER 4
#define SUS1 5
#define SUS2 6

#define BSPD_BrakePressed_Pin GPIO_PIN_13
#define BSPD_BrakePressed_GPIO_Port GPIOC
#define BSPD_MotorOn_Pin GPIO_PIN_14
#define BSPD_MotorOn_GPIO_Port GPIOC
#define BSPD_Shutdown_Pin GPIO_PIN_15
#define BSPD_Shutdown_GPIO_Port GPIOC
#define BSPD_BrakeFailure_Pin GPIO_PIN_3
#define BSPD_BrakeFailure_GPIO_Port GPIOC
#define BSPD_MotorFailure_Pin GPIO_PIN_2
#define BSPD_MotorFailure_GPIO_Port GPIOB

typedef struct AnalogVoltages {
    float apps1, apps2;
    float bse1, bse2;
    float steer;
    float sus1, sus2;
} AnalogVoltages;

typedef struct BSPD {
    bool brake_pressed;
    bool motor_on;
    bool shutdown;
    bool brake_failure;
    bool motor_failure;
} BSPD;

static uint16_t adcValues[NUM_SENSORS * NUM_VALUES_PER_SENSOR];

/**
 * Calibrate the offset for the analog sensors.
 * @param hadc1
 * @return error code
 */
int adc_calibrate(ADC_HandleTypeDef* hadc1);

/**
 * Initialize the ADC and DMA for analog sensors.
 * @param hadc1
 * @return error code
 */
int adc_start(ADC_HandleTypeDef* hadc1);

/**
 * Stop the ADC and DMA for analog sensors.
 * @param hadc1
 * @return error code
 */
int adc_stop(ADC_HandleTypeDef* hadc1);

/**
 * Update the values in the struct with the latest voltages.
 * @param analogVoltages
 */
void adc_get(AnalogVoltages* analogVoltages);
/**
 * Get the state of the BSPD to be sent to telemetry.
 * @param bspd_state
 */
void adc_get_bspd(BSPD* bspd_state);

#endif //VCU_FIRMWARE_2024_ANALOG_H
