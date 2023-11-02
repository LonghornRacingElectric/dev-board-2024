//
// Created by yashk on 10/29/2023.
//

#ifndef VCU_FIRMWARE_2024_IMU_INTERNAL_H
#define VCU_FIRMWARE_2024_IMU_INTERNAL_H

#include "stm32h7xx_hal.h"
#include "VcuModel.h"
#include "library.h"

unsigned int Get_IMU_Response(SPI_HandleTypeDef* hspi, VcuInput* input);

unsigned int Send_IMU_Request(SPI_HandleTypeDef* hspi);



unsigned int Calibrate_EEPROM(SPI_HandleTypeDef* hspi);

bool ping_IMU(UART_HandleTypeDef* huart);

bool ping_EEPROM(UART_HandleTypeDef* huart);

#endif //VCU_FIRMWARE_2024_IMU_INTERNAL_H
