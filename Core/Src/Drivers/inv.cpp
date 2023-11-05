//
// Created by yashk on 10/29/2023.
//

#include "inv.h"
#include "InterruptHandlers.h"
#include "firmware_faults.h"
#include <cmath>

using namespace std;

unsigned int Init_CAN_INV(FDCAN_HandleTypeDef* hfdcan){
  return 0;
}

unsigned int update_INV(VcuInput* input, FDCAN_HandleTypeDef* hfdcan) {
  int8_t module_Atemp = (INVTemp1Data[0] << 8) + INVTemp1Data[1];
  int8_t module_Btemp = (INVTemp1Data[2] << 8) + INVTemp1Data[3];
  int8_t module_Ctemp = (INVTemp1Data[4] << 8) + INVTemp1Data[5];
  input->inverterTemp = max(module_Atemp * 10.0f, max(module_Btemp * 10.0f, module_Ctemp * 10.0f));
  input->motorTemp = ((INVTemp3Data[4] << 8) + INVTemp3Data[5]) * 10.0f;

  uint64_t result_state, result_fault = 0x0;
  for (int i = 7; i >= 0; i--) {
    result_state = (result_state << 8) | INVStateData[i];
    result_fault = (result_fault << 8) | INVFaultData[i];
  }
  inv_state_vector = result_state;
  inv_fault_vector = result_fault;

  input->inverterReady = (bool) (inv_fault_vector == 0x0);
  return 0;
}