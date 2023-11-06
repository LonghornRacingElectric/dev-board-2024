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
  int8_t module_Atemp = (INVTemp1Data[1] << 8) + INVTemp1Data[0];
  int8_t module_Btemp = (INVTemp1Data[3] << 8) + INVTemp1Data[2];
  int8_t module_Ctemp = (INVTemp1Data[5] << 8) + INVTemp1Data[4];
  input->inverterTemp = max(module_Atemp / 10.0f, max(module_Btemp / 10.0f, module_Ctemp / 10.0f));
  input->motorTemp = ((INVTemp3Data[5] << 8) + INVTemp3Data[4]) / 10.0f;

  uint64_t result_state = 0x0, result_fault = 0x0;
  for (int i = 0; i < 8; i++) {
    result_state += (uint64_t) INVStateData[i] << (8 * i);
    result_fault += (uint64_t) INVFaultData[i] << (8 * i);
  }
  inv_state_vector = result_state;
  inv_fault_vector = result_fault;

  /* We need to check whether inv_fault_vector is useless or not */
  // input->inverterReady = (bool) (inv_fault_vector == 0x0);


  return 0;
}

void Interpret_INV_State(std::string &vsm_state, std::string &inverter_state){
  switch(inv_state_vector & 0x0FF){
    case 0x0:
      vsm_state = "Startup";
      break;
    case 0x1:
      vsm_state = "Pre-charge Init";
      break;
    case 0x2:
      vsm_state = "Pre-charge Active";
      break;
    case 0x3:
      vsm_state = "Pre-charge Complete";
      break;
    case 0x4:
      vsm_state = "VSM Wait";
      break;
    case 0x5:
      vsm_state = "VSM Run";
      break;
    case 0x6:
      vsm_state = "Motor Running";
      break;
    case 0x7:
      vsm_state = "Blink Fault Code";
      break;
    case 0xE:
      vsm_state = "Shutdown";
      break;
    case 0xF:
      vsm_state = "Recycle Power";
      break;
    default:
      vsm_state = "Unknown State";
      break;
  }
  switch((inv_state_vector & 0x0FF0000) >> 16){
    case 0x0:
      inverter_state = "Power On";
      break;
    case 0x1:
      inverter_state = "Stop";
      break;
    case 0x2:
      inverter_state = "Open Loop";
      break;
    case 0x3:
      inverter_state = "Closed Loop";
      break;
    case 0x4:
      inverter_state = "Wait";
      break;
    case 0x8:
      inverter_state = "Idle Run";
      break;
    case 0x9:
      inverter_state = "Idle Stop";
      break;
    default:
      inverter_state = "Internal States";
      break;
  }
}

void Interpret_INV_Fault(std::vector<std::string> &faults){
  if(inv_fault_vector == 0x0){
    return;
  }
  if(inv_fault_vector & 0x1){
    faults.emplace_back("HW Gate/Desaturation Fault");
  }
  if(inv_fault_vector & 0x2){
    faults.emplace_back("HW Over-current Fault");
  }
  if(inv_fault_vector & 0x4){
    faults.emplace_back("Accelerator Shorted");
  }
  if(inv_fault_vector & 0x8){
    faults.emplace_back("Accelerator Open");
  }
  if(inv_fault_vector & 0x10){
    faults.emplace_back("Current Sensor Low");
  }
  if(inv_fault_vector & 0x20){
    faults.emplace_back("Current Sensor High");
  }
  if(inv_fault_vector & 0x40){
    faults.emplace_back("Module Temperature Low");
  }
  if(inv_fault_vector & 0x80){
    faults.emplace_back("Module Temperature High");
  }

  if(inv_fault_vector & 0x100){
    faults.emplace_back("Control PCB Temperature Low");
  }
  if(inv_fault_vector & 0x200){
    faults.emplace_back("Control PCB Temperature High");
  }
  if(inv_fault_vector & 0x400){
    faults.emplace_back("Gate Drive PCB Temperature Low");
  }
  if(inv_fault_vector & 0x800){
    faults.emplace_back("Gate Drive PCB Temperature High");
  }
  if(inv_fault_vector & 0x1000){
    faults.emplace_back("5V Sense Voltage Low");
  }
  if(inv_fault_vector & 0x2000){
    faults.emplace_back("5V Sense Voltage High");
  }
  if(inv_fault_vector & 0x4000){
    faults.emplace_back("12V Sense Voltage Low");
  }
  if(inv_fault_vector & 0x8000){
    faults.emplace_back("12V Sense Voltage High");
  }
  if(inv_fault_vector & 0x10000){
    faults.emplace_back("2.5V Sense Voltage Low");
  }
  if(inv_fault_vector & 0x20000){
    faults.emplace_back("2.5V Sense Voltage High");
  }
  if(inv_fault_vector & 0x40000){
    faults.emplace_back("1.5V Sense Voltage Low");
  }
  if(inv_fault_vector & 0x80000){
    faults.emplace_back("1.5V Sense Voltage High");
  }
  if(inv_fault_vector & 0x100000){
    faults.emplace_back("DC Bus Voltage High");
  }
  if(inv_fault_vector & 0x200000){
    faults.emplace_back("DC Bus Voltage Low");
  }
  if(inv_fault_vector & 0x400000){
    faults.emplace_back("Pre-charge Timeout");
  }
  if(inv_fault_vector & 0x800000){
    faults.emplace_back("Pre-charge Voltage Failure");
  }
  if(inv_fault_vector & 0x1000000){
    faults.emplace_back("EEPROM Checksum Invalid");
  }
  if(inv_fault_vector & 0x2000000){
    faults.emplace_back("EEPROM Data Out of Range");
  }
  if(inv_fault_vector & 0x4000000){
    faults.emplace_back("EEPROM Update Required");
  }
  if(inv_fault_vector & 0x8000000){
    faults.emplace_back("Hardware DC Bus Over-Voltage during initialization ");
  }
  if(inv_fault_vector & 0x10000000){
    faults.emplace_back("Gate Driver initialization");
  }
  if(inv_fault_vector & 0x20000000){
    faults.emplace_back("Reserved (no fault)");
  }
  if(inv_fault_vector & 0x40000000){
    faults.emplace_back("Brake Shorted");
  }
  if(inv_fault_vector & 0x80000000){
    faults.emplace_back("Brake Open");
  }

  uint32_t new_inv_fault_vector = inv_fault_vector >> 32;
  if(new_inv_fault_vector & 0x1){
    faults.emplace_back("Motor Over-speed Fault");
  }
  if(new_inv_fault_vector & 0x2){
    faults.emplace_back("Over-current Fault");
  }
  if(new_inv_fault_vector & 0x4){
    faults.emplace_back("Over-voltage Fault");
  }
  if(new_inv_fault_vector & 0x8){
    faults.emplace_back("Inverter Over-temperature Fault");
  }
  if(new_inv_fault_vector & 0x10){
    faults.emplace_back("Accelerator Input Shorted");
  }
  if(new_inv_fault_vector & 0x20){
    faults.emplace_back("Accelerator Input Open");
  }
  if(new_inv_fault_vector & 0x40){
    faults.emplace_back("Direction Command Fault");
  }
  if(new_inv_fault_vector & 0x80){
    faults.emplace_back("Inverter Reponse Time-out Fault");
  }
  if(new_inv_fault_vector & 0x100){
    faults.emplace_back("HW Gate/Desaturation Fault");
  }
  if(new_inv_fault_vector & 0x200){
    faults.emplace_back("HW Over-current Fault");
  }
  if(new_inv_fault_vector & 0x400){
    faults.emplace_back("Under-voltage Fault");
  }
  if(new_inv_fault_vector & 0x800){
    faults.emplace_back("CAN Command Message Lost Fault");
  }
  if(new_inv_fault_vector & 0x1000){
    faults.emplace_back("Motor Over-temperature Fault");
  }
  if(new_inv_fault_vector & 0x2000){
    faults.emplace_back("Reserved (no fault)");
  }
  if(new_inv_fault_vector & 0x4000){
    faults.emplace_back("Reserved (no fault)");
  }
  if(new_inv_fault_vector & 0x8000){
    faults.emplace_back("Reserved (no fault)");
  }
  if(new_inv_fault_vector & 0x10000){
    faults.emplace_back("Brake Input Shorted Fault");
  }
  if(new_inv_fault_vector & 0x20000){
    faults.emplace_back("Brake Input Open Fault");
  }
  if(new_inv_fault_vector & 0x40000){
    faults.emplace_back("Module A Over-temperature Fault");
  }
  if(new_inv_fault_vector & 0x80000){
    faults.emplace_back("Module B Over-temperature Fault");
  }
  if(new_inv_fault_vector & 0x100000){
    faults.emplace_back("Module C Over-temperature Fault");
  }
  if(new_inv_fault_vector & 0x200000){
    faults.emplace_back("PCB Over-temperature Fault");
  }
  if(new_inv_fault_vector & 0x400000){
    faults.emplace_back("Gate Drive Board 1 Over-temperature Fault");
  }
  if(new_inv_fault_vector & 0x800000){
    faults.emplace_back("Gate Drive Board 2 Over-temperature Fault");
  }
  if(new_inv_fault_vector & 0x1000000){
    faults.emplace_back("Gate Drive Board 3 Over-temperature Fault");
  }
  if(new_inv_fault_vector & 0x2000000){
    faults.emplace_back("Current Sensor Fault");
  }
  if(new_inv_fault_vector & 0x4000000){
    faults.emplace_back("Gate Drive Over-voltage");
  }
  if(new_inv_fault_vector & 0x8000000){
    faults.emplace_back("Reserved (no fault)");
  }
  if(new_inv_fault_vector & 0x10000000){
    faults.emplace_back("Hardware DC Bus Over-Voltage Fault");
  }
  if(new_inv_fault_vector & 0x20000000){
    faults.emplace_back("Reserved (no fault)");
  }
  if(new_inv_fault_vector & 0x40000000){
    faults.emplace_back("Resolver Not Connected");
  }
  if(new_inv_fault_vector & 0x80000000){
    faults.emplace_back("Reserved (no fault)");
  }
}