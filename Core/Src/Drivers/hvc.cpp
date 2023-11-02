//
// Created by yashk on 10/29/2023.
//

#include "hvc.h"
#include "InterruptHandlers.h"
#include "firmware_faults.h"

using namespace std;

unsigned int update_HVC(VcuInput* input) {
  if(HVCData[2] == 0){
    return 1;
  }
  input->batteryTemp = HVCData[0];
  input->batterySoc = HVCData[1];
  return 0;
}
