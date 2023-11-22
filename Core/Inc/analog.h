//
// Created by yashk on 10/29/2023.
//

#ifndef VCU_FIRMWARE_2024_ANALOG_H
#define VCU_FIRMWARE_2024_ANALOG_H

#include "stm32h7xx_hal.h"
#include "VcuModel.h"

unsigned int Init_Analog(ADC_HandleTypeDef *hadc1);

unsigned int Get_Analog(VcuInput* input, VcuParameters* params);

#endif //VCU_FIRMWARE_2024_ANALOG_H
