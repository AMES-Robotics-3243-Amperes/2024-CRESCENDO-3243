// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.Constants;
import frc.robot.Constants.Climber;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
  boolean motorOneComplete;
  boolean motorTwoComplete;

  /** Creates a new ClimberSubsystem. */
  public SubsystemClimber() {
    // ££ Creates the two motors
    motorOne = new CANSparkMax(Constants.Climber.ClimberConstants.IDs.kMotorOne, MotorType.kBrushless);
    motorTwo = new CANSparkMax(Constants.Climber.ClimberConstants.IDs.kMotorTwo, MotorType.kBrushless);

    // ££ Creates the PID controllers
    motorOneController = motorOne.getPIDController();
    motorTwoController = motorTwo.getPIDController();

    // ££ Sets PID values
    setPIDValues(motorOneController,
      Constants.Climber.ClimberConstants.ClimberPIDFF.kP,
      Constants.Climber.ClimberConstants.ClimberPIDFF.kI,
      Constants.Climber.ClimberConstants.ClimberPIDFF.kD,
      Constants.Climber.ClimberConstants.ClimberPIDFF.kFF);

    // ££ Creates the relative encoders
    motorOneRelativeEncoder = motorOne.getEncoder();
    motorTwoRelativeEncoder = motorTwo.getEncoder();

    motorOneRelativeEncoder.setPositionConversionFactor(Constants.Climber.ClimberConstants.kGearRatio);
    motorTwoRelativeEncoder.setPositionConversionFactor(Constants.Climber.ClimberConstants.kGearRatio);

    // // ££ Sets the feedback devices
    // motorOneController.setFeedbackDevice(motorOneRelativeEncoder);
    // motorTwoController.setFeedbackDevice(motorTwoRelativeEncoder);
  }

  public void setMotorPositionTarget(double position) {
    motorOneController.setReference(position, ControlType.kPosition);
    motorTwoController.setReference(position, ControlType.kPosition);
  }

  public void runClimber(boolean dPadUp, boolean dPadDown) {
    if (motorOne.getOutputCurrent() > Constants.Climber.ClimberConstants.MotorCurrentLimit) {
      motorOneComplete = true;
    }

    if (motorTwo.getOutputCurrent() > Constants.Climber.ClimberConstants.MotorCurrentLimit) {
      motorTwoComplete = true;
    }

    if (dPadUp) {
      motorOne.set(Constants.Climber.ClimberConstants.MotorSpeeds.kRiseSpeed);
      motorTwo.set(Constants.Climber.ClimberConstants.MotorSpeeds.kRiseSpeed);
    } 
    
    if (dPadDown) {
      if (!motorOneComplete) {
        motorOne.set(Constants.Climber.ClimberConstants.MotorSpeeds.kInitialFallSpeed);
      }

      if (!motorTwoComplete) {
      motorTwo.set(Constants.Climber.ClimberConstants.MotorSpeeds.kInitialFallSpeed);
      }
    }

    if (motorOneComplete && motorTwoComplete) {
      double currentRotations = motorOneRelativeEncoder.getPosition();
      double targetPositon = currentRotations - Constants.Climber.ClimberConstants.kPositionOffset;

      setMotorPositionTarget(targetPositon);
    }
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
