#ifndef _imu_receiver_h_
#define _imu_receiver_h_

#include "xdainterface.h"

#include <iostream>
#include <stdexcept>
#include <string>

using std::chrono::milliseconds;

int imu_init();
void imu_stop();
bool getImuDataFrame(Eigen::Vector3d &acc, Eigen::Vector3d &gyro, Eigen::Quaterniond &quat);

#endif
