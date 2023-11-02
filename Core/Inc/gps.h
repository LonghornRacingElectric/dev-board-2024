//
// Created by yashk on 10/29/2023.
//

#ifndef VCU_FIRMWARE_2024_GPS_H
#define VCU_FIRMWARE_2024_GPS_H

#include "stm32h7xx_hal.h"
#include "VcuModel.h"
#include "library.h"

unsigned int Get_GPS_Response(UART_HandleTypeDef* huart, VcuInput* input);

unsigned int Send_GPS_Request(UART_HandleTypeDef* huart);

bool ping_GPS(UART_HandleTypeDef* huart);

#endif //VCU_FIRMWARE_2024_GPS_H
