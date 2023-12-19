#include "pdu.h"
#include "pdu.h"
#include "angel_can.h"
#include "faults.h"

static CanRx lvbatt_status_mailbox;
static CanRx thermal_status_mailbox;
static uint32_t pdu_sendBrakeLight(float brightnessLeft, float brightnessRight, uint32_t delta);
static uint32_t pdu_sendBuzzer(BuzzerType buzzerType, uint32_t delta);
static uint32_t pdu_sendCoolingOutput(uint16_t radiatorFanRpm, float pumpPercentage, uint32_t delta);

// Note that all the IDs PDU interacts with will be mentioned here
void pdu_init(){
    can_addMailbox(PDU_VCU_LVBAT, &lvbatt_status_mailbox);
    can_addMailbox(PDU_VCU_THERMAL, &thermal_status_mailbox);
    can_addTimer(VCU_PDU_BRAKELIGHT, 10);
    can_addTimer(VCU_PDU_BUZZER, 10);
    can_addTimer(VCU_PDU_COOLING, 250);
}

static uint32_t pdu_sendBrakeLight(float brightnessLeft, float brightnessRight, uint32_t delta) {
    uint8_t data[8];
    data[0] = (uint8_t) (brightnessLeft * 255.0f);
    data[1] = (uint8_t) (brightnessRight * 255.0f);
    return can_send_periodic(VCU_PDU_BRAKELIGHT, 2, data, delta);
}

static uint32_t pdu_sendBuzzer(BuzzerType buzzerType, uint32_t delta) {
    uint8_t data[1];
    data[0] = (uint8_t) buzzerType;
    return can_send_periodic(VCU_PDU_BUZZER, 1, data, delta);
}

static uint32_t pdu_sendCoolingOutput(float radiatorFanRpm, float pumpPercentage, uint32_t delta) {
    uint8_t data[3];
    can_writeBytes(data, 0, 1, (uint16_t) radiatorFanRpm); // max will occur at 8300 rpm, but higher will just be ignored
    data[2] = (uint8_t) pumpPercentage;
    return can_send_periodic(VCU_PDU_COOLING, 3, data, delta);
}

uint32_t pdu_update(PDUStatus* status, VcuOutput* vcuOutput, float deltaTime){
    uint32_t t = deltaTime * 1000;
    volatile uint32_t error_accum = 0;

    // TODO update vcu core to output brake lights
    // TODO pdu_sendBrakeLight(vcuOutput->brakeLightLeft)
    error_accum += pdu_sendBuzzer(vcuOutput->r2dBuzzer ? BUZZER_BUZZ : BUZZER_SILENT, t);
    // TODO update vcu core to output cooling percentages
    // TODO pdu_sendCoolingOutput(vcuOutput->radiatorFanRpm, vcuOutput->pumpPercentage);
    if(error_accum == 1) FAULT_SET(&vcu_fault_vector, FAULT_VCU_PDU);

    if(lvbatt_status_mailbox.isRecent){
        lvbatt_status_mailbox.isRecent = false;
        status->lvVoltage = (float) can_readBytes(lvbatt_status_mailbox.data, 0, 1) / 10.0f; // in 0.1V units, range is 0.0 - 28.0V
        status->lvSoC = (float) can_readBytes(lvbatt_status_mailbox.data, 2, 3) / 10.0f; // in 0.1% units
        status->lvCurrent = (float) can_readBytes(lvbatt_status_mailbox.data, 4, 5) / 100.0f; // in 0.01A units, range is 0.00 - 100.00A
        status->isRecent = true;
    }
    if(thermal_status_mailbox.isRecent){
        thermal_status_mailbox.isRecent = false;
        status->volumetricFlowRate = (float) thermal_status_mailbox.data[0] / 10.0f; // range from 0.0 to 25.5 L/min
        status->waterTempMotor = (float) thermal_status_mailbox.data[1]; // degrees C
        status->waterTempInverter = (float) thermal_status_mailbox.data[2]; // degrees C
        status->waterTempRadiator = (float) thermal_status_mailbox.data[3]; // degrees C
        status->radiatorFanRpm = (float) can_readBytes(thermal_status_mailbox.data, 4, 5) / 10.0f;
        status->isRecent = true;
    }
    return error_accum;
}