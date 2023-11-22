//
// Created by yashk on 10/29/2023.
//

#include "inv.h"
#include "faults.h"
#include "can.h"
#include <cmath>
using namespace std;


static CanRx voltageMailbox;
static CanRx currentMailbox;
static CanRx motorTempMailbox;
static CanRx inverterTempMailbox;
static CanRx motorPosMailbox;
static CanRx inverterStateMailbox;
static CanRx inverterFaultMailbox;
static CanRx torqueInfoMailbox;
static CanRx highSpeedMailbox;
static CanRx paramsResponseMailbox;

static CanRx test_torqueRequestMailbox;



void inverter_init() {
    // TODO implement

    // example:
    can_addMailbox(INV_VOLTAGE, 0x7FF, &voltageMailbox);
    can_addMailbox(INV_CURRENT, 0x7FF,&currentMailbox);
    can_addMailbox(INV_TEMP3_DATA, 0x7FF,&motorTempMailbox);
    can_addMailbox(INV_TEMP1_DATA, 0x7FF,&inverterTempMailbox);
    can_addMailbox(INV_MOTOR_POSITIONS, 0x7FF,&motorPosMailbox);
    can_addMailbox(INV_STATE_CODES, 0x7FF,&inverterStateMailbox);
    can_addMailbox(INV_FAULT_CODES, 0x7FF,&inverterFaultMailbox);
    can_addMailbox(INV_TORQUE_TIMER, 0x7FF,&torqueInfoMailbox);
    // can_addMailbox(INV_HIGH_SPEED_MSG, 0x7FF,&highSpeedMailbox);
    can_addMailbox(VCU_INV_COMMAND, 0x7FF,&test_torqueRequestMailbox);
    can_addMailbox(INV_VCU_PARAMS_RESPONSE, 0x7FF,&paramsResponseMailbox);

}

float inverter_getStatus(InverterStatus* status) {

    if(inverterTempMailbox.isRecent) {
        auto temp = can_readBytes(inverterTempMailbox.data, 0, 1) +
                    can_readBytes(inverterTempMailbox.data, 2, 3) +
                    can_readBytes(inverterTempMailbox.data, 4, 5);
        status->inverterTemp = (float) temp / 30.0f;
        inverterTempMailbox.isRecent = false;
        status->isRecent = true;
    }

    if(motorTempMailbox.isRecent) {
        status->motorTemp = (float) can_readBytes(motorTempMailbox.data, 4, 5) / 10.0f;
        motorTempMailbox.isRecent = false;
        status->isRecent = true;
    }

    if(motorPosMailbox.isRecent) {
        status->motorAngle = (float) can_readBytes(motorPosMailbox.data, 0, 1) / 10.0f;
        status->rpm = (float) can_readBytes(motorPosMailbox.data, 2, 3); //Out of all of these, idk why this isnt shifted
        status->inverterFrequency = (float) can_readBytes(motorPosMailbox.data, 4, 5) / 10.0f;
        status->resolverAngle = (float) can_readBytes(motorPosMailbox.data, 6, 7) / 10.0f;
        motorPosMailbox.isRecent = false;
        status->isRecent = true;
    }

    if(voltageMailbox.isRecent) {
        status->voltage = (float) can_readBytes(voltageMailbox.data, 0, 1) / 10.0f;
        status->outputVoltage = (float) can_readBytes(voltageMailbox.data, 2, 3) / 10.0f;
        status->ABVoltage = (float) can_readBytes(voltageMailbox.data, 4, 5) / 10.0f;
        status->BCVoltage = (float) can_readBytes(voltageMailbox.data, 6, 7) / 10.0f;
        voltageMailbox.isRecent = false;
        status->isRecent = true;
    }

    if(currentMailbox.isRecent) {
        status->phaseACurrent = (float) can_readBytes(currentMailbox.data, 0, 1) / 10.0f;
        status->phaseBCurrent = (float) can_readBytes(currentMailbox.data, 2, 3) / 10.0f;
        status->phaseCCurrent = (float) can_readBytes(currentMailbox.data, 4, 5) / 10.0f;
        status->current = (float) can_readBytes(currentMailbox.data, 6, 7) / 10.0f;
        currentMailbox.isRecent = false;
        status->isRecent = true;
    }

    if(inverterStateMailbox.isRecent) {
        status->stateVector = can_readBytes(inverterStateMailbox.data, 0, 7);
        inverterStateMailbox.isRecent = false;
        status->isRecent = true;
    }

    if(inverterFaultMailbox.isRecent) {
        status->faultVector = can_readBytes(inverterFaultMailbox.data, 0, 1);
        inverterFaultMailbox.isRecent = false;
        status->isRecent = true;
    }

    if(torqueInfoMailbox.isRecent) {
        status->torqueActual = (float) can_readBytes(torqueInfoMailbox.data, 0, 1) / 10.0f;
        status->torqueCommand = (float) can_readBytes(torqueInfoMailbox.data, 2, 3) / 10.0f;
        torqueInfoMailbox.isRecent = false;
        status->isRecent = true;
    }

    if(paramsResponseMailbox.isRecent) {
        auto data = (uint16_t) can_readBytes(paramsResponseMailbox.data, 4, 5);
        if(paramsResponseMailbox.data[2] == 0){
            vcu_fault_vector |= FAULT_VCU_INVPARAMS;
        }
        paramsResponseMailbox.isRecent = false;
    }

    if(test_torqueRequestMailbox.isRecent) {
        float test_torque_command = (float) can_readBytes(test_torqueRequestMailbox.data, 0, 1) / 10.0f;
        test_torqueRequestMailbox.isRecent = false;
        return test_torque_command;
    }
    return 0.0;

}

unsigned int inverter_sendTorqueCommand(float torque, float rpm, bool enable_inverter) {
    static uint8_t torque_command[8];
    auto tc = (int16_t) (torque * 10.0f);
    auto sc = (int16_t) rpm;
    can_writeBytes(torque_command, 0, 1, tc);
    can_writeBytes(torque_command, 2, 3, sc);
    torque_command[5] = (uint8_t) enable_inverter; // Enable

    return can_send(VCU_INV_COMMAND, 8, torque_command);
}

unsigned int inverter_resetFaults() {
    // send a can message telling the inverter to reset faults by setting addr 20 to 0
    return inverter_paramsIO(20, 0, true);
}

unsigned int inverter_paramsIO(uint16_t param_addr, uint16_t param_value, bool write){
    // send a can message telling the inverter to set params
    static uint8_t set_params[8];
    can_writeBytes(set_params, 0, 1, param_addr); //param addr
    can_writeBytes(set_params, 4, 5, param_value);  //param value
    set_params[2] = (uint8_t) write; //sets to write mode

    return can_send(VCU_INV_PARAMS_REQUEST, 8, set_params);
}

void Interpret_INV_State(InverterStatus* status, std::string &vsm_state, std::string &inverter_state){
  switch(status->stateVector & 0x0FF){
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
  switch((status->stateVector & 0x0FF0000) >> 16){
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