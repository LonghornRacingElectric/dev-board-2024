//
// Created by yashk on 10/29/2023.
//

#ifndef VCU_FIRMWARE_2024_INV_H
#define VCU_FIRMWARE_2024_INV_H

#include "VcuModel.h"
#include "stm32h7xx_hal.h"
#include "library.h"

unsigned int Init_CAN_INV(FDCAN_HandleTypeDef* hfdcan);

unsigned int update_INV(VcuInput* input, FDCAN_HandleTypeDef* hfdcan);

#endif //VCU_FIRMWARE_2024_INV_H
