//
// Created by yashk on 10/22/2023.
//

#ifndef VCU_FIRMWARE_2024_GLOBALS_H
#define VCU_FIRMWARE_2024_GLOBALS_H

#include <cstdint>
#include "stm32h7xx_hal.h"

static uint32_t global_shutdown = 0;

typedef struct bspd {
  int BrakePressed;
  int BrakeBroken;
  int MotorON;
  int MotorPressed;
  int BSPDShutdown;
} BSPD;

int Critical_Error_Handler(uint32_t fault_type);
int Noncritical_Error_Handler(uint32_t fault_type);
void init_TX(FDCAN_TxHeaderTypeDef*, uint32_t);

#endif //VCU_FIRMWARE_2024_GLOBALS_H
