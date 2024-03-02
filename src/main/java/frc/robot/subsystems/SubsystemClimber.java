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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber.ClimberConstants.ClimberPIDFF;
import frc.robot.Constants.Climber.ClimberConstants.IDs;

public class SubsystemClimber extends SubsystemBase {
  // ££ Initialize climber motors
  CANSparkMax motorOne;
  CANSparkMax motorTwo;

  // ££ Initializes the PID controllers
  private final SparkPIDController motorOneController;
  private final SparkPIDController motorTwoController;

  // ££ Initializes the relative encoders
  RelativeEncoder motorOneRelativeEncoder;
  RelativeEncoder motorTwoRelativeEncoder;

  // ££ Boolean values for if the climber arms have reached positions ready to pull up
  boolean motorOneComplete = false;
  boolean motorTwoComplete = false;

  // ££ Initializes the limit switches
  DigitalInput limitSwitchOne;
  DigitalInput limitSwitchTwo;

  double currentRotationsOne;
  double targetPositionOne;
  double currentRotationsTwo;
  double targetPositionTwo;

  boolean positionInitializedOne;
  boolean positionInitializedTwo;
  
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
    setPIDValues(motorTwoController, 
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

    // ££ Sets limit switches
    limitSwitchOne = new DigitalInput(IDs.kSwitchOne);
    limitSwitchTwo = new DigitalInput(IDs.kSwitchTwo);

    motorOne.setInverted(false);
    motorTwo.setInverted(true);

    targetPositionOne = 0;
    targetPositionTwo = 0;

    motorOne.setSmartCurrentLimit(SmartMotorCurrentLimit);
    motorTwo.setSmartCurrentLimit(SmartMotorCurrentLimit);
  }

  public boolean runClimber(boolean dPadUp, boolean dPadDown) {
    if (motorOne.getOutputCurrent() > MotorCurrentLimit) {
      motorOneComplete = true;
    }

    if (motorTwo.getOutputCurrent() > MotorCurrentLimit) {
      motorTwoComplete = true;
    }

      currentRotationsOne = motorOneRelativeEncoder.getPosition();
      currentRotationsTwo = motorTwoRelativeEncoder.getPosition();

    if (dPadUp) {
      // if (motorOneRelativeEncoder.getPosition() >= motorPositionLimit) {
      //   motorOne.set(0);
      // } else {
      //   motorOne.set(MotorSpeeds.kRiseSpeed);
      // }

      // if (motorTwoRelativeEncoder.getPosition() >= motorPositionLimit) {
      //   motorTwo.set(0);
      // } else {
      //   motorTwo.set(MotorSpeeds.kRiseSpeed);
      // }



      // if (currentRotationsOne < motorPositionLimit) {
      //   targetPositionOne = currentRotationsOne + 4;
      // }
      // if (currentRotationsTwo < motorPositionLimit) {
      //   targetPositionTwo = currentRotationsTwo + 4;
      // }

      motorTwo.set(MotorSpeeds.kRiseSpeed);

    } 
      // :> test
    if (dPadDown) {
      // if (motorOneRelativeEncoder.getPosition() <= 0.0) {
      //   motorOne.set(0);
      // } else {
      //   if (!motorOneComplete) {
      //   motorOne.set(MotorSpeeds.kInitialFallSpeed);
      //   } else {
      //    currentRotationsOne = motorOneRelativeEncoder.getPosition();
      //    targetPositionOne = currentRotationsOne + kPositionOffset;
      //    motorOneController.setReference(targetPositionOne, ControlType.kPosition);

      //   }
      // }

      // if (motorTwoRelativeEncoder.getPosition() <= 0.0) {
      //   motorTwo.set(0);
      // } else {
      //   if (!motorTwoComplete) {
      //   motorTwo.set(MotorSpeeds.kInitialFallSpeed);
      //   } else {
      //     currentRotationsTwo = motorTwoRelativeEncoder.getPosition();
      //     targetPositionTwo = currentRotationsTwo + kPositionOffset;
      //     motorTwoController.setReference(targetPositionTwo, ControlType.kPosition);

      //   }
      // }

     
      // if (currentRotationsOne >= 0 || !positionInitializedOne) {
      //   targetPositionOne = currentRotationsOne - 4;
      
      // }
      // if (currentRotationsTwo >= 0 || !positionInitializedTwo) {
      //   targetPositionTwo = currentRotationsTwo - 4;
      // }

      if (!limitSwitchTwo.get()) {
        motorTwo.set(MotorSpeeds.kInitialFallSpeed);
      } else {
        motorTwo.set(0);
      }

      // if (motorOne.getOutputCurrent() < MotorCurrentLimit) {
      //   currentRotationsOne = 0;
      // }
      // if (motorTwo.getOutputCurrent() < MotorCurrentLimit) {

      // }

      // Plan
      // 1. Lower the climber until the current is not above a certain amount
    }

    // motorOneController.setReference(targetPositionOne, ControlType.kPosition);
    // motorTwoController.setReference(targetPositionTwo, ControlType.kPosition);



    if (motorOneComplete && motorTwoComplete) {
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

  // ££ If the climbers are fully lowered (limit switches activated) sets the base encoder value
  public void calibrateClimber(boolean limitSwitchOneTripped, boolean limitSwitchTwoTripped) {
    
    if (limitSwitchOneTripped) {
      if (!positionInitializedOne || motorOneRelativeEncoder.getPosition() <= 0) {
        targetPositionOne = 0;
      }
      motorOneRelativeEncoder.setPosition(0);
      positionInitializedOne = true;
      motorOneController.setReference(targetPositionOne, ControlType.kPosition);
    }
    
    
    if (limitSwitchTwoTripped) {
      if (!positionInitializedTwo || motorTwoRelativeEncoder.getPosition() <= 0) {
        targetPositionTwo = 0;
      }
      motorTwoRelativeEncoder.setPosition(0);
      positionInitializedTwo = true;
      motorTwoController.setReference(targetPositionTwo, ControlType.kPosition);
    }
  }

  // ££ Auto climber
  public boolean autoRunClimber() {
    boolean finished = false;

    if (motorOneRelativeEncoder.getPosition() < motorPositionLimit || motorTwoRelativeEncoder.getPosition() < motorPositionLimit) {
      runClimber(true, false);
    } else {
      finished = runClimber(false, true);
    }

    return finished;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // ££ Calibrates climber
    calibrateClimber(limitSwitchOne.get(), limitSwitchTwo.get());
  }
}
