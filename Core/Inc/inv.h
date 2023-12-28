#ifndef DEV_BOARD_2024_INV_H
#define DEV_BOARD_2024_INV_H

// TODO wip
#include "stm32h7xx_hal.h"
#include <string>

typedef struct InverterStatus {
    bool isRecent;
    float voltage; //input voltage into DC side
    float current; //input current into DC side
    float rpm;
    float inverterTemp;
    float motorTemp;

    float motorAngle;
    float resolverAngle;

    float phaseACurrent;
    float phaseBCurrent;
    float phaseCCurrent;

    float BCVoltage;
    float ABVoltage;
    float outputVoltage;

    float inverterFrequency;

    float torqueActual;
    float torqueCommand;

    uint64_t faultVector;
    uint64_t stateVector;
    // ...
} InverterStatus;

/**
 * Initialize inverter CAN communication.
 */
void inverter_init();

/**
 * Get latest inverter info.
 */
void inverter_update(InverterStatus* status);

/**
 * Send a torque command over CAN.
 */
unsigned int inverter_sendTorqueCommand(float torque, float rpm, bool enable_inverter, uint32_t deltaTime);

/**
 * Reset faults over CAN.
 * @return error_code
 */
unsigned int inverter_resetFaults();

/**
 * Set Torque Limit over CAN.
 */
unsigned int inverter_setTorqueLimit(uint32_t torqueLimit);

/**
 * Selectively Enable/Disable messages that inverter sends over CAN.
 * The mask allows the VCU to understand all necessary messages that the inverter will send.
 * See https://app.box.com/s/vf9259qlaadhzxqiqrt5cco8xpsn84hk/file/27334613044 for more details
 */
unsigned int inverter_enableFaults(uint32_t faultMask);

/**
 * Set the CAN bitrate for the inverter.
 * @param bitrate
 * @return
 */
unsigned int inverter_updateCANBitRate(uint32_t bitrate);

/**
 * Read/write inverter parameters over CAN.
 * @param param_addr
 * @param param_value
 * @param write
 * @return error_code
 */
unsigned int inverter_paramsIO(uint16_t param_addr, uint16_t param_value, bool write);

#endif //DEV_BOARD_2024_INV_H
