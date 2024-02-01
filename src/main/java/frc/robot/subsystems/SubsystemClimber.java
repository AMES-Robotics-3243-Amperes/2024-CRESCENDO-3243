// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import static frc.robot.Constants.Climber.ClimberConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemClimber extends SubsystemBase {
  // ££ Initialize climber motors
  CANSparkMax motorOne;
  CANSparkMax motorTwo;

  // ££ Initializes the PID controllers
  private final SparkPIDController motorOneController;
  private final SparkPIDController motorTwoController;

  // ££ Initializes the relative encoders
  private final RelativeEncoder motorOneRelativeEncoder;
  private final RelativeEncoder motorTwoRelativeEncoder;

  // ££ Boolean values for if the climber arms have reached positions ready to pull up
  boolean motorOneComplete = false;
  boolean motorTwoComplete = false;

  /** Creates a new ClimberSubsystem. */
  public SubsystemClimber() {
    // ££ Creates the two motors
    motorOne = new CANSparkMax(IDs.kMotorOne, MotorType.kBrushless);
    motorTwo = new CANSparkMax(IDs.kMotorTwo, MotorType.kBrushless);

    // ££ Creates the PID controllers
    motorOneController = motorOne.getPIDController();
    motorTwoController = motorTwo.getPIDController();

    // ££ Sets PID values
    setPIDValues(motorOneController,
      ClimberPIDFF.kP,
      ClimberPIDFF.kI,
      ClimberPIDFF.kD,
      ClimberPIDFF.kFF);

    // ££ Creates the relative encoders
    motorOneRelativeEncoder = motorOne.getEncoder();
    motorTwoRelativeEncoder = motorTwo.getEncoder();

    motorOneRelativeEncoder.setPositionConversionFactor(kGearRatio);
    motorTwoRelativeEncoder.setPositionConversionFactor(kGearRatio);

    // ££ Sets the feedback devices
    motorOneController.setFeedbackDevice(motorOneRelativeEncoder);
    motorTwoController.setFeedbackDevice(motorTwoRelativeEncoder);
  }

  public void setMotorPositionTarget(double position) {
    motorOneController.setReference(position, ControlType.kPosition);
    motorTwoController.setReference(position, ControlType.kPosition);
  }

  public boolean runClimber(boolean dPadUp, boolean dPadDown) {
    if (motorOne.getOutputCurrent() > MotorCurrentLimit) {
      motorOneComplete = true;
    }

    if (motorTwo.getOutputCurrent() > MotorCurrentLimit) {
      motorTwoComplete = true;
    }

    if (dPadUp) {
      motorOne.set(MotorSpeeds.kRiseSpeed);
      motorTwo.set(MotorSpeeds.kRiseSpeed);

      motorOneComplete = true;
      motorTwoComplete = true;
    } 
    
    if (dPadDown) {
      if (!motorOneComplete) {
        motorOne.set(MotorSpeeds.kInitialFallSpeed);
      }

      if (!motorTwoComplete) {
      motorTwo.set(MotorSpeeds.kInitialFallSpeed);
      }
    }

    if (motorOneComplete && motorTwoComplete) {
      double currentRotations = motorOneRelativeEncoder.getPosition();
      double targetPosition = currentRotations - kPositionOffset;

      setMotorPositionTarget(targetPosition);
      return true;
    }

    return false;
  }

  public void setPIDValues(SparkPIDController PIDController, double p, double i, double d, double ff) {
    PIDController.setP(p);
    PIDController.setI(i);
    PIDController.setD(d);
    PIDController.setFF(ff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
