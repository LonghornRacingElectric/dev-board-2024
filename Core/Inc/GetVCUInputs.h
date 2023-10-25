//
// Created by yashk on 10/16/2023.
//

#ifndef VCU_FIRMWARE_2024_GETVCUINPUTS_H
#define VCU_FIRMWARE_2024_GETVCUINPUTS_H

#include "firmware_faults.h"
#include "VcuModel.h"
#include "library.h"
#include "stm32h7xx_hal.h"

#define ADC_BUF_SIZE 5
#define APPS1_CHANNEL 0
#define APPS2_CHANNEL 1
#define BSE1_CHANNEL 2
#define BSE2_CHANNEL 3
#define STEER_CHANNEL 4

static uint16_t adcData[ADC_BUF_SIZE];


uint32_t Get_VCU_Inputs(VcuInput* , VcuParameters*, ADC_HandleTypeDef* , FDCAN_HandleTypeDef* , SPI_HandleTypeDef* , UART_HandleTypeDef* );
int update_HVC(VcuInput* );
int update_INV(VcuInput* );
int update_PDU(VcuInput* );
int update_WHS(VcuInput* );
int update_IMU(VcuInput* );

#endif //VCU_FIRMWARE_2024_GETVCUINPUTS_H
