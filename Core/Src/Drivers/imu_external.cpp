//
// Created by yashk on 10/29/2023.
//

#include "imu_external.h"
#include "InterruptHandlers.h"
#include "firmware_faults.h"
#include <cmath>

using namespace std;
float IMUACCELSCALAR = 0;
float IMUGYROSCALAR = 0;

int update_IMU(VcuInput* input){
  if(IMU2Data[12] == 0){
    return 1;
  }
  uint16_t imu2AccelX = IMU2Data[0] << 8 | IMU2Data[1];
  uint16_t imu2AccelY = IMU2Data[2] << 8 | IMU2Data[3];
  uint16_t imu2AccelZ = IMU2Data[4] << 8 | IMU2Data[5];
  uint16_t imu2GyroX = IMU2Data[6] << 8 | IMU2Data[7];
  uint16_t imu2GyroY = IMU2Data[8] << 8 | IMU2Data[9];
  uint16_t imu2GyroZ = IMU2Data[10] << 8 | IMU2Data[11];

  input->imu2Accel = {imu2AccelX * IMUACCELSCALAR, imu2AccelY * IMUACCELSCALAR, imu2AccelZ * IMUACCELSCALAR};
  input->imu2Gyro = {imu2GyroX * IMUGYROSCALAR, imu2GyroY * IMUGYROSCALAR, imu2GyroZ * IMUGYROSCALAR};

  if(IMU2Data[12] == 0){
    return 1;
  }
  uint16_t imu3AccelX = IMU3Data[0] << 8 | IMU3Data[1];
  uint16_t imu3AccelY = IMU3Data[2] << 8 | IMU3Data[3];
  uint16_t imu3AccelZ = IMU3Data[4] << 8 | IMU3Data[5];
  uint16_t imu3GyroX = IMU3Data[6] << 8 | IMU3Data[7];
  uint16_t imu3GyroY = IMU3Data[8] << 8 | IMU3Data[9];
  uint16_t imu3GyroZ = IMU3Data[10] << 8 | IMU3Data[11];

  input->imu3Accel = {imu3AccelX * IMUACCELSCALAR, imu3AccelY * IMUACCELSCALAR, imu3AccelZ * IMUACCELSCALAR};
  input->imu3Gyro = {imu3GyroX * IMUGYROSCALAR, imu3GyroY * IMUGYROSCALAR, imu3GyroZ * IMUGYROSCALAR};



  return 0;
}
