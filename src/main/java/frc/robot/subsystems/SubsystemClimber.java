// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubsystemClimber extends SubsystemBase {
  // ££ Initialize climber motors
  CANSparkMax motorOne;
  CANSparkMax motorTwo;

  /** Creates a new ClimberSubsystem. */
  public SubsystemClimber() {
    motorOne = new CANSparkMax(Constants.Climber.ClimberConstants.IDs.kMotorOne, MotorType.kBrushless);
    motorTwo = new CANSparkMax(Constants.Climber.ClimberConstants.IDs.kMotorTwo, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
