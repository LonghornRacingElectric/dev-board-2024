//
// Created by yashk on 11/5/2023.
//

#include "firmware_faults.h"

uint32_t vcu_fault_vector = 0; //will be sent by VCU and updated occasionally

uint64_t inv_fault_vector = 0; //will be sent by Inverter and updated occasionally
uint64_t inv_state_vector = 0; //will be sent by Inverter and updated occasionally

uint32_t hvc_fault_vector = 0; //will be sent by HVC and updated occasionally

uint32_t pdu_fault_vector = 0; //will be sent by PDU and updated occasionally

void set_fault(uint32_t fault) { vcu_fault_vector |= fault; }
void clear_fault(uint32_t fault) { vcu_fault_vector &= ~fault; }
void clear_all_faults() { vcu_fault_vector = 0x0; }
