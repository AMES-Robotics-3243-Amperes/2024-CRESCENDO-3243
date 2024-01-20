// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.networktables.GenericEntry;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.IntakePIDs;

public class SubsystemIntake extends SubsystemBase {

  // :> Creates the pivotMotor
  protected final CANSparkMax touronMotor = new CANSparkMax(Constants.IntakeConstants.touronMotor, MotorType.kBrushless);
  // :> Creates the pivot PIDController
  protected final SparkPIDController touronPID;
  // :> Creates the pivot AbsoluteEncoder
  protected final SparkAbsoluteEncoder touronAbsoluteEncoder; 

  private final CANSparkMax intakeMotor1;

  protected final RelativeEncoder IntakeEncoder;

  protected final SparkPIDController IntakePID;

  // :> Shuffleboard entries for us to be able to tune PIDs live
  protected GenericEntry touronP;
  protected GenericEntry touronI;
  protected GenericEntry touronD;
  protected GenericEntry touronFF;

  // :> Creates the enum type to be able to pass in a setpoint from a command
  public enum setPoints{
    position1 (Constants.IntakeConstants.touronSetPoint1),
    position2 (Constants.IntakeConstants.touronSetPoint2),
    position3 (Constants.IntakeConstants.touronSetPoint3);

    public final double angle;
    setPoints(double angle) {
      this.angle = angle;
    }
  }
  // :> Creates Shuffleboard tab to be able to put stuff on it relating to Intake
  protected ShuffleboardTab tab = Shuffleboard.getTab("Intake Tuning");


  /** Creates a new IntakeSubsystem. */
  public SubsystemIntake() {

    intakeMotor1 = new CANSparkMax(IntakeMotorID, CANSparkMax.MotorType.kBrushless);
    IntakeEncoder = intakeMotor1.getEncoder();
    IntakePID = intakeMotor1.getPIDController();
    IntakePID.setFeedbackDevice(IntakeEncoder);

    IntakePID.setP(kP);
    IntakePID.setI(kI);
    IntakePID.setD(kD);
    IntakePID.setFF(kFF);

    IntakePID.setReference(0, CANSparkMax.ControlType.kVelocity);
    // :> Gets the absolute encoder from the motor
    touronAbsoluteEncoder = touronMotor.getAbsoluteEncoder(Type.kDutyCycle);
    // :> Gets the PIDController from the motor
    touronPID = touronMotor.getPIDController();
    // :> Accounts for the amount of turns it takes for the motor to actually move the intake
    touronAbsoluteEncoder.setPositionConversionFactor(Constants.IntakeConstants.touronConversionFactor);
    //:> Sets the PIDController to take in data from the absolute encoder when doing its calculations
    touronPID.setFeedbackDevice(touronAbsoluteEncoder);

    setTouronPIDFValues(touronPID, IntakePIDs.touronP, IntakePIDs.touronI, IntakePIDs.touronD, IntakePIDs.touronFF);

    // 
    // :> Shuffleboard PID Tuning
    //
    touronP = tab.add("TRN P Value:", touronP).getEntry();
    touronI = tab.add("TRN I Value:", touronI).getEntry();
    touronD = tab.add("TRN D Value:", touronD).getEntry();
      
    /* :> Sets the idlemode to break, 
      *   the reason why we do this is to make it so when the intake stops getting input it doesn't flail about
    */
    touronMotor.setIdleMode(IdleMode.kBrake);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
    * Sets the PIDValues of the Touron when called
    * @param pidController
    * @param p
    * @param i
    * @param d
    * @param f
    * @author :>
    */
  protected void setTouronPIDFValues(SparkPIDController pidController, double p, double i, double d, double f) {
    pidController.setP(p);
    pidController.setP(i);
    pidController.setP(d);
    pidController.setP(f);
  }
  /**
    * gets the current position of the Touron
    * Currently only planned to be used for auto if used at all
    * @return Position of the Touron
    * @author :>
    */
  public double getTouronMotorPosition() {
    return touronAbsoluteEncoder.getPosition();
  }
  /**
    * Sets the position of the Touron based off of an inputted setPoint
    * @param position
    * @author :>
    */
  public void setPositionReference(setPoints position) {
    touronPID.setReference(position.angle, ControlType.kPosition);
  }

  public void TurnOnIntake() {
    IntakePID.setReference(kV, CANSparkMax.ControlType.kVelocity);
  }

  public void TurnOffIntake() {
    IntakePID.setReference(0, CANSparkMax.ControlType.kVelocity);
  }
}
