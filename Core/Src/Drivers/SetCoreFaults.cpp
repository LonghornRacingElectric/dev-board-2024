//
// Created by yashk on 10/16/2023.
//

#include "SetCoreFaults.h"
#include "globals.h"

uint32_t Set_Core_Faults(VcuOutput* output) {
    if(output->faultApps){
      return Critical_Error_Handler(APPS_FAULT);
    }
    if(output->faultBse){
      return Critical_Error_Handler(BSE_FAULT);
    }
    if(output->faultStompp){
      return Critical_Error_Handler(STOMPP_FAULT);
    }
    if(output->faultSteering){
      return Noncritical_Error_Handler(STEERING_FAULT);
    }

    //TODO: stuff for track positioning, traction control, torque map

    return 0;
}