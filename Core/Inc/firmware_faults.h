//
// Created by yashk on 10/15/2023.
//

#ifndef VCU_FIRMWARE_2024_FIRMWARE_FAULTS_H
#define VCU_FIRMWARE_2024_FIRMWARE_FAULTS_H

#include <cstdint>

extern uint32_t vcu_fault_vector; //will be sent by VCU and updated occasionally

extern uint64_t inv_fault_vector; //will be sent by Inverter and updated occasionally
extern uint64_t inv_state_vector; //will be sent by Inverter and updated occasionally

extern uint32_t hvc_fault_vector; //will be sent by HVC and updated occasionally

extern uint32_t pdu_fault_vector; //will be sent by PDU and updated occasionally

//Core Faults which are derived from VCU CORE Outputs, see Libs/VcuCore/src/VcuModel.h
#define APPS_FAULT 0x00000001
#define BSE_FAULT 0x00000002
#define STOMPP_FAULT 0x00000004
#define TORQUE_MAP_FAULT 0x00000008
#define TRACTION_CONTROL_FAULT 0x00000010
#define TRACK_POSITIONING_FAULT 0x00000020
#define STEERING_FAULT 0x0000040

//Write faults where data cannot be trasmitted
#define DRS_DATA_FAULT 0x00000100 //Will not be used cause Aero can't do it, sad
#define TORQUEREQUEST_DATA_FAULT 0x00000200
#define CARDIAGNOSTICS_DATA_FAULT 0x00000400

//Read faults where data is corrupted, not received, or unable to be requested
#define ADC_DATA_FAULT 0x00010000
#define IMU_DATA_FAULT 0x00080000
#define WHEELSPEED_DATA_FAULT 0x00100000 //activates if 2+ wheelspeeds are unable to be read
#define CELLULAR_DATA_FAULT 0x01000000
#define GPS_DATA_FAULT 0x02000000
#define INVERTER_DATA_FAULT 0x04000000
#define HVC_DATA_FAULT 0x08000000
#define PDU_DATA_FAULT 0x10000000
#define DASH_DATA_FAULT 0x20000000
#define GENERIC_CAN_DATA_FAULT 0x80000000
#define VCU_DATA_FAULT 0x40000000

#define VCU_FAULT_LIST vcu_fault_vector
void set_fault(uint32_t fault);
void clear_fault(uint32_t fault);
void clear_all_faults();



#endif //VCU_FIRMWARE_2024_FIRMWARE_FAULTS_H
