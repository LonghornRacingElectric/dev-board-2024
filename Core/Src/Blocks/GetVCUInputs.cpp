/*
 * GetVCUInputs.cpp
 *
 * Receives data from the following components and stores them in VcuInput
 * Checks whether data can be received from the following components and sets faults accordingly
 * ADCs: apps1, apps2, bse1, bse2, steerpot
 * SPI: onboard-imu, eeprom (CS, MOSI, MISO, CLK)
 * - Onboard IMU: accel1, gyro1
 * UART: gps (UART_TX, UART_RX)
 * - GPS: lat, long, speed, heading
 * - NOTE: could be replaced with cell module.
 * CAN: inverter, hvc, pdu, whs1, whs2, whs3, whs4 (CAN_TX, CAN_RX)
 * - Inverter: inverter temp, inverter ready, motor temp
 * - HVC: battery temp, battery soc, accel2, gyro2 (IMU2)
 * - WHS: fl, fr, bl, br
 * - PDU: accel3, gyro3 (IMU2)
 * GPIO: drive switch (DASH), inverter reset (DASH), calibration (Master Switch ???), shutdown (from BSPD)
 *
 * Also decodes input data and places it in meaningful form, along with logic using GPIO pins.
 */


#include "GetVCUInputs.h"
#include "InterruptHandlers.h"
#include "globals.h"
#include <algorithm>
#include <cstdio>

using namespace std;

float IMUACCELSCALAR = 0;
float IMUGYROSCALAR = 0;

char TxUARTCmds[UART_BUF_SIZE] = {0};
char RxUARTCmds[UART_BUF_SIZE] = {0};

uint8_t TxSPI[SPI_BUF_SIZE] = {0};
uint8_t RxSPI[SPI_BUF_SIZE] = {0};


uint32_t Get_VCU_Inputs(VcuInput* input,
                        VcuParameters* params,
                        ADC_HandleTypeDef* hadc,
                        FDCAN_HandleTypeDef* hfdcan,
                        SPI_HandleTypeDef* hspi,
                        UART_HandleTypeDef* huart){

  //Request data from IMU
  if(HAL_SPI_Transmit_IT(hspi, (uint8_t*)TxSPI, SPI_BUF_SIZE) != HAL_OK){
    return Critical_Error_Handler(IMU_DATA_FAULT);
  }

  //Send out VCU Request Data
  sprintf((char*)Tx0Data, "urmom");
  init_TX(&Tx0Header, VCU_REQUEST_DATA_ID);
  if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &Tx0Header, reinterpret_cast<uint8_t *>(Tx0Data)) != HAL_OK){
    return Critical_Error_Handler(VCU_DATA_FAULT);
  }

  // Request data from GPS UART (or cell module)
  sprintf((char*)TxUARTCmds, "Give me the data");
  if(HAL_UART_Transmit_IT(huart, (uint8_t*)TxUARTCmds, UART_BUF_SIZE) != HAL_OK){
    return Critical_Error_Handler(GPS_DATA_FAULT);
  }

  input->apps1 = adcData[APPS1_CHANNEL] * params->apps1VoltageMax / 65536.0;
  input->apps2 = adcData[APPS2_CHANNEL] * params->apps2VoltageMax / 65536.0;
  input->bse1 = adcData[BSE1_CHANNEL] * params->bseVoltageMax / 65536.0;
  input->bse2 = adcData[BSE2_CHANNEL] * params->bseVoltageMax / 65536.0;
  input->steeringWheelPotVoltage = adcData[STEER_CHANNEL] * 5.0 / 65536.0;

  // Next with UART
  // Start DMA with UART now

  if(HAL_UART_Receive_IT(huart, (uint8_t*)RxUARTCmds, UART_BUF_SIZE) != HAL_OK){
    return Critical_Error_Handler(GPS_DATA_FAULT);
  }

  return 0;
  // Set data from HVC, INV, PDU, WHS from CAN
  if(update_HVC(input) != 0){
    set_fault(HVC_DATA_FAULT);
    return 1;
  }
  if(update_INV(input) != 0){
    set_fault(INVERTER_DATA_FAULT);
    return 1;
  }
  if(update_PDU(input) != 0){
    set_fault(PDU_DATA_FAULT);
  }
  if(update_WHS(input) != 0){
    set_fault(WHEELSPEED_DATA_FAULT);
  }
  if(update_IMU(input) != 0){
    set_fault(IMU_DATA_FAULT);
  }

  // Next with SPI (synchronous)
  HAL_SPI_Receive_IT(hspi, (uint8_t*)RxSPI, SPI_BUF_SIZE);






  // Finally with GPIO, and commit logic based on this
    return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
  // Checks if the interrupt is from the correct UART
}

int update_HVC(VcuInput* input){
  if(HVCData[2] == 0){
    return 1;
  }
  input->batteryTemp = HVCData[0];
  input->batterySoc = HVCData[1];
  return 0;
}

int update_INV(VcuInput* input){
  int8_t module_Atemp = (INVTemp1Data[0] << 8) + INVTemp1Data[1];
  int8_t module_Btemp = (INVTemp1Data[2] << 8) + INVTemp1Data[3];
  int8_t module_Ctemp = (INVTemp1Data[4] << 8) + INVTemp1Data[5];
  input->inverterTemp = max(module_Atemp * 10.0f, max(module_Btemp * 10.0f, module_Ctemp * 10.0f));
  input->motorTemp = ((INVTemp3Data[4] << 8) + INVTemp3Data[5]) * 10.0f;

  uint64_t result_state, result_fault = 0x0;
  for(int i = 7; i >= 0; i--){
    result_state = (result_state << 8) | INVStateData[i];
    result_fault = (result_fault << 8) | INVFaultData[i];
  }
  inv_state_vector = result_state;
  inv_fault_vector = result_fault;

  input->inverterReady = (bool) (inv_fault_vector == 0x0);
  return 0;
}

int update_PDU(VcuInput* input){
  if(PDUData[0] == 0){
    return 1;
  }
  return 0;
}

int update_WHS(VcuInput* input){
  return 0;
}

int update_IMU(VcuInput* input){
  if(IMU2Data[12] == 0){
    return 1;
  }
  uint16_t imu2AccelX = IMU2Data[0] << 8 | IMU2Data[1];
  uint16_t imu2AccelY = IMU2Data[2] << 8 | IMU2Data[3];
  uint16_t imu2AccelZ = IMU2Data[4] << 8 | IMU2Data[5];
  uint16_t imu2GyroX = IMU2Data[6] << 8 | IMU2Data[7];
  uint16_t imu2GyroY = IMU2Data[8] << 8 | IMU2Data[9];
  uint16_t imu2GyroZ = IMU2Data[10] << 8 | IMU2Data[11];

  input->imu2Accel = {imu2AccelX * IMUACCELSCALAR, imu2AccelY * IMUACCELSCALAR, imu2AccelZ * IMUACCELSCALAR};
  input->imu2Gyro = {imu2GyroX * IMUGYROSCALAR, imu2GyroY * IMUGYROSCALAR, imu2GyroZ * IMUGYROSCALAR};

  if(IMU2Data[12] == 0){
    return 1;
  }
  uint16_t imu3AccelX = IMU3Data[0] << 8 | IMU3Data[1];
  uint16_t imu3AccelY = IMU3Data[2] << 8 | IMU3Data[3];
  uint16_t imu3AccelZ = IMU3Data[4] << 8 | IMU3Data[5];
  uint16_t imu3GyroX = IMU3Data[6] << 8 | IMU3Data[7];
  uint16_t imu3GyroY = IMU3Data[8] << 8 | IMU3Data[9];
  uint16_t imu3GyroZ = IMU3Data[10] << 8 | IMU3Data[11];

  input->imu3Accel = {imu3AccelX * IMUACCELSCALAR, imu3AccelY * IMUACCELSCALAR, imu3AccelZ * IMUACCELSCALAR};
  input->imu3Gyro = {imu3GyroX * IMUGYROSCALAR, imu3GyroY * IMUGYROSCALAR, imu3GyroZ * IMUGYROSCALAR};



  return 0;
}
