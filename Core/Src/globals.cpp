//
// Created by yashk on 10/22/2023.
//

#include "globals.h"
#include "firmware_faults.h"

motor_info_t motorInfo = {0};
inverter_info_t inverterInfo = {0};
inv_params_t can_response = {0};
inv_params_t torque_limit_response = {0};

int Critical_Error_Handler(uint32_t fault_type){
  global_shutdown = 1;
  set_fault(fault_type);
  return 1;
}

int Noncritical_Error_Handler(uint32_t fault_type) {
  set_fault(fault_type);
  return 0;
}

void init_TX(FDCAN_TxHeaderTypeDef* TxHeader, uint32_t identifier){
  TxHeader->Identifier = identifier;
  TxHeader->IdType = FDCAN_STANDARD_ID;
  TxHeader->TxFrameType = FDCAN_DATA_FRAME;
  TxHeader->DataLength = FDCAN_DLC_BYTES_8;
  TxHeader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader->BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader->FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader->MessageMarker = 0;
}
