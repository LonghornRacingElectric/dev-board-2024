//
// Created by yashk on 10/29/2023.
//

#ifndef VCU_FIRMWARE_2024_ANALOG_H
#define VCU_FIRMWARE_2024_ANALOG_H

#include "stm32h7xx_hal.h"
#include "VcuModel.h"
#include "library.h"

#define APPS1_CHANNEL 0
#define APPS2_CHANNEL 1
#define BSE1_CHANNEL 2
#define BSE2_CHANNEL 3
#define STEER_CHANNEL 4

static uint16_t adcData[5] = {0};

unsigned int Init_Analog(ADC_HandleTypeDef *hadc);

unsigned int Get_Analog(ADC_HandleTypeDef *hadc, VcuInput* input, VcuParameters* params);

#endif //VCU_FIRMWARE_2024_ANALOG_H
