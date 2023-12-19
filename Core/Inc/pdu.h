
#ifndef DEV_BOARD_2024_PDU_H
#define DEV_BOARD_2024_PDU_H
#include <cstdint>
#include "VcuModel.h"

enum BuzzerType {
    BUZZER_SILENT = 0,
    BUZZER_BUZZ = 1,
    BUZZER_DEEP_IN_THE_HEART_OF_TEXAS = 2,
};

struct PDUStatus {
    float volumetricFlowRate;
    float waterTempInverter;
    float waterTempMotor;
    float waterTempRadiator;

    float radiatorFanRpm;

    float lvVoltage;
    float lvSoC;
    float lvCurrent;

    bool isRecent = false;
};

/**
 * Initialize the PDU's CAN mailboxes and wire signals
 */
void pdu_init();

/**
 * Update the PDU's status based on CAN messages and VCU Core outputs.
 * @param status
 * @param vcuOutput
 * @param deltaTime
 * @return error_code
 */
uint32_t pdu_update(PDUStatus* status, VcuOutput* vcuOutput, float deltaTime);


#endif //DEV_BOARD_2024_PDU_H
