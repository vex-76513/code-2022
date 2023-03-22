#include "main.h"
#include "ports.hpp"
#pragma once

using namespace okapi::literals;

class DrivetrainClass {
public:
  okapi::Motor topLeftMotor = DR_TL_PORT, topRightMotor = -DR_TR_PORT,
               bottomRightMotor = -DR_BR_PORT, bottomLeftMotor = DR_BL_PORT;

  okapi::IterativeVelPIDController YAW_CONTROLLER =
      okapi::IterativeControllerFactory::velPID(
          .0001, 0, 1. / 45, 0,
          okapi::VelMathFactory::createPtr(
              360, std::make_unique<okapi::AverageFilter<3>>()));

  pros::IMU imu = pros::IMU(IMU_PORT);

  DrivetrainClass() {}

  void fieldorientedtoraw(double &xSpeed, double &forwardSpeed) {
    auto theta = imu.get_rotation();

    auto xSpeednew = -forwardSpeed * sin(theta * 1_pi / 180) +
                     xSpeed * cos(theta * 1_pi / 180);
    forwardSpeed = xSpeed * sin(theta * 1_pi / 180) +
                   forwardSpeed * cos(theta * 1_pi / 180);

    xSpeed = xSpeednew;
  }

  void arcade(double xSpeed, double forwardSpeed, double yaw) {
    YAW_CONTROLLER.setTarget(yaw * 45); // bot goes to 45 rpm max?????

    yaw = YAW_CONTROLLER.step(imu.get_rotation());

    auto maxVoltage = 600;
    topLeftMotor.moveVelocity(static_cast<int16_t>(
        std::clamp(-forwardSpeed - xSpeed - yaw, -1.0, 1.0) * maxVoltage));
    topRightMotor.moveVelocity(static_cast<int16_t>(
        std::clamp(-forwardSpeed + xSpeed + yaw, -1.0, 1.0) * maxVoltage));
    bottomRightMotor.moveVelocity(static_cast<int16_t>(
        std::clamp(forwardSpeed + xSpeed - yaw, -1.0, 1.0) * maxVoltage));
    bottomLeftMotor.moveVelocity(static_cast<int16_t>(
        std::clamp(forwardSpeed - xSpeed + yaw, -1.0, 1.0) * maxVoltage));
  }
} Drivetrain;
