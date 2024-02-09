// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemShooter extends SubsystemBase {
  /** Creates a new SubsystemShooter. */

  private CANSparkMax flywheelMotorLeft = new CANSparkMax(0, MotorType.kBrushless);
  //TODO: set the device ID to whatever the device ID is

  private CANSparkMax flywheelMotorRight = new CANSparkMax( 0, MotorType.kBrushless);
  //TODO: set the device ID to whatever the device ID is
 
  // && Declares PID object
  private SparkPIDController flywheelPID;

  public SubsystemShooter() {

    flywheelPID = flywheelMotorLeft.getPIDController();

    setFlywheelPID();

    setFollow();

  }

  // && Method to set right motor to follow left motor
  public void setFollow(){
    flywheelMotorRight.follow(flywheelMotorLeft);
  }

  // && Method to set flywheel speed
  public void setFlywheelSpeed(double velocity){
    flywheelPID.setReference(velocity, ControlType.kVelocity);

  }

  // && Method to detect the flywheel speed
  public double getFlywheelSpeed(){
    double flyWheelSpeed = flywheelMotorLeft.getEncoder().getVelocity();
    return flyWheelSpeed;
  }

  // && Method to stop the flywheel
  public void stopFlywheel(){
    flywheelMotorLeft.set(0.0);
    
  }

  // && Method to set the P, I, and D, values for the flywheel PID
  // && TODO: set actual values for P, I, and D
  public void setFlywheelPID(){
    flywheelPID.setP(0);
    flywheelPID.setI(0);
    flywheelPID.setD(0);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
