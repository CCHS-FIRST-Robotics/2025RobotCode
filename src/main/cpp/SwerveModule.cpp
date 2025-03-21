// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

SwerveModule::SwerveModule(const int driveMotorChannel, const int turningMotorChannel, const int driveEncoderChannelA, const int driveEncoderChannelB, const int turningEncoderChannelA, const int turningEncoderChannelB)
        :m_driveMotor(driveMotorChannel),
        m_turningMotor(turningMotorChannel),
        m_driveEncoder(driveEncoderChannelA, driveEncoderChannelB),
        m_turningEncoder(turningEncoderChannelA, turningEncoderChannelB) {

  m_driveEncoder.SetDistancePerPulse(2 * std::numbers::pi * kWheelRadius / kEncoderResolution);

  // angle/pulse for turn encoder
  m_turningEncoder.SetDistancePerPulse(2 * std::numbers::pi / kEncoderResolution);

  // Limit pid controller input range btwn -pi and pi
  m_turningPIDController.EnableContinuousInput(-units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});
}

frc::SwerveModuleState SwerveModule::GetState() const {
  return {units::meters_per_second_t{m_driveEncoder.GetRate()},
          units::radian_t{m_turningEncoder.GetDistance()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return {units::meter_t{m_driveEncoder.GetDistance()}, units::radian_t{m_turningEncoder.GetDistance()}};
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState& referenceState) {
  frc::Rotation2d encoderRotation{
      units::radian_t{m_turningEncoder.GetDistance()}};

  referenceState.Optimize(encoderRotation);

  // smoother
  referenceState.CosineScale(encoderRotation);

  // calculate outputs from pid controllers
  const auto driveOutput = m_drivePIDController.Calculate(m_driveEncoder.GetRate(), referenceState.speed.value());

  const auto driveFeedforward = m_driveFeedforward.Calculate(referenceState.speed);

  const auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t{m_turningEncoder.GetDistance()},
      referenceState.angle.Radians());

  const auto turnFeedforward = m_turnFeedforward.Calculate(m_turningPIDController.GetSetpoint().velocity);

  // Set motor outputs
  m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
  m_turningMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
}
