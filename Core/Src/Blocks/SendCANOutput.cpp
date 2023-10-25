//
// Created by yashk on 10/16/2023.
//

#include "SendCANOutput.h"

FDCAN_TxHeaderTypeDef CAN_OUT_TxHeader;

#define VCU_INV_COMMAND 0x0C0 //Stores inverter command
#define VCU_INV_PARAMETER_RW 0x0C1 //Sets inverter parameter r/w
#define VCU_PDU_BRAKING 0x0F0 //Stores PDU braking data

#define VCU_DASH_FAULT_INFO 0x200 //Sends out VCU fault info to dash
#define VCU_DASH_CAR_INFO 0x204 //Sends out typical dash information (e.g. speed, acceleration, SoC, etc...) to dash


int8_t CAN_OUT_TxData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t DASHCAN_OUT_TxData[8] = {0, 0, 0, 0, 0, 0, 0, 0};
using namespace std;

uint32_t Send_CAN_Output(VcuInput* input, VcuOutput* output, VcuParameters* params, BSPD* bspd, FDCAN_HandleTypeDef* hfdcan) {

  //Send out Torque Command
  auto torqueCommand = (int16_t) output->inverterTorqueRequest;
  CAN_OUT_TxData[0] = (int8_t) (torqueCommand >> 8);
  CAN_OUT_TxData[1] = (int8_t) (torqueCommand & 0x0FF);
  CAN_OUT_TxData[4] = 1;
  CAN_OUT_TxData[5] = 0x01; //Maybe change this to depend on global_shutdown and inverterReady
  //TODO: Add torque limit later

  init_TX(&CAN_OUT_TxHeader, VCU_INV_COMMAND);
  if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &CAN_OUT_TxHeader, reinterpret_cast<uint8_t *>(CAN_OUT_TxData)) != HAL_OK){
    return Critical_Error_Handler(TORQUEREQUEST_DATA_FAULT);
  }

  CAN_OUT_TxData[0] = output->r2dBuzzer;
  CAN_OUT_TxData[1] = output->prndlState;
  CAN_OUT_TxData[2] = output->enableDragReduction;
  CAN_OUT_TxData[3] = bspd->BrakePressed;
  CAN_OUT_TxData[4] = bspd->BrakeBroken;
  CAN_OUT_TxData[5] = bspd->MotorON;
  CAN_OUT_TxData[6] = bspd->MotorPressed;
  CAN_OUT_TxData[7] = 0xFF;

  init_TX(&CAN_OUT_TxHeader, VCU_PDU_BRAKING);
  if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &CAN_OUT_TxHeader, reinterpret_cast<uint8_t *>(CAN_OUT_TxData)) != HAL_OK){
    return Critical_Error_Handler(PDU_DATA_FAULT);
  }

  //Send out VCU Fault Info
  uint32_t bit_mask = 0x000000FF;
  for(int i = 0; i < 4; i++){
    DASHCAN_OUT_TxData[i] = (uint8_t) vcu_fault_vector & bit_mask;
    bit_mask = bit_mask << 8;
  }
  init_TX(&CAN_OUT_TxHeader, VCU_DASH_FAULT_INFO);
  if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &CAN_OUT_TxHeader, reinterpret_cast<uint8_t *>(DASHCAN_OUT_TxData)) != HAL_OK){
    return Critical_Error_Handler(DASH_DATA_FAULT);
  }

  //Send out VCU Car Info
  DASHCAN_OUT_TxData[0] = (uint8_t) hypot(output->vehicleVelocity.x, output->vehicleVelocity.y, output->vehicleVelocity.z);
  DASHCAN_OUT_TxData[1] = (uint8_t) hypot(output->vehicleAcceleration.x, output->vehicleAcceleration.y, output->vehicleAcceleration.z);
  DASHCAN_OUT_TxData[2] = (uint8_t) input->batterySoc;
  DASHCAN_OUT_TxData[3] = (uint8_t) input->batteryTemp;

  //Everything else are booleans, we can represent them in a single byte but I am currently too lazy to implement right now
  DASHCAN_OUT_TxData[4] = (uint8_t) bspd->MotorON;
  DASHCAN_OUT_TxData[5] = (uint8_t) bspd->BrakePressed;
  DASHCAN_OUT_TxData[6] = (uint8_t) output->prndlState;
  DASHCAN_OUT_TxData[7] = (uint8_t) output->enableDragReduction;

  init_TX(&CAN_OUT_TxHeader, VCU_DASH_CAR_INFO);
  if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &CAN_OUT_TxHeader, reinterpret_cast<uint8_t *>(DASHCAN_OUT_TxData)) != HAL_OK){
    return Critical_Error_Handler(DASH_DATA_FAULT);
  }
  return 0;
}