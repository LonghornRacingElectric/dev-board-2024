//
// Created by yashk on 10/16/2023.
//

#ifndef VCU_FIRMWARE_2024_SENDCANOUTPUT_H
#define VCU_FIRMWARE_2024_SENDCANOUTPUT_H

#include "firmware_faults.h"
#include "globals.h"
#include "VcuModel.h"
#include "library.h"
#include "stm32h7xx_hal.h"
#include <cmath>

uint32_t Send_CAN_Output(VcuInput* , VcuOutput* , VcuParameters* , BSPD* , FDCAN_HandleTypeDef* );
uint32_t Update_CAN_Parameters(uint16_t param_addr, uint32_t param_value, FDCAN_HandleTypeDef* hfdcan, bool write);

#endif //VCU_FIRMWARE_2024_SENDCANOUTPUT_H
