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
  protected final CANSparkMax m_PivotMotor = new CANSparkMax(Constants.IntakeConstants.touronMotor, MotorType.kBrushless);
  // :> Creates the pivot PIDController
  protected final SparkPIDController m_PivotPID;
  // :> Creates the pivot AbsoluteEncoder
  protected final SparkAbsoluteEncoder m_PivotAbsoluteEncoder; 

  protected final CANSparkMax m_IntakeMotor;

  protected final RelativeEncoder m_IntakeRelativeEncoder;

  protected final SparkPIDController m_IntakePID;

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


  /** Creates a new SubsystemIntake. */
  public SubsystemIntake() {

    m_IntakeMotor = new CANSparkMax(IntakeMotorID, CANSparkMax.MotorType.kBrushless);
    m_IntakeRelativeEncoder = m_IntakeMotor.getEncoder();
    m_IntakePID = m_IntakeMotor.getPIDController();
    m_IntakePID.setFeedbackDevice(m_IntakeRelativeEncoder);

    m_IntakePID.setP(kP);
    m_IntakePID.setI(kI);
    m_IntakePID.setD(kD);
    m_IntakePID.setFF(kFF);

    m_IntakePID.setReference(0, CANSparkMax.ControlType.kVelocity);
    // :> Gets the absolute encoder from the motor
    m_PivotAbsoluteEncoder = m_PivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    // :> Gets the PIDController from the motor
    m_PivotPID = m_PivotMotor.getPIDController();
    // :> Accounts for the amount of turns it takes for the motor to actually move the intake
    m_PivotAbsoluteEncoder.setPositionConversionFactor(Constants.IntakeConstants.touronConversionFactor);
    //:> Sets the PIDController to take in data from the absolute encoder when doing its calculations
    m_PivotPID.setFeedbackDevice(m_PivotAbsoluteEncoder);

    setPivotPIDFValues(m_PivotPID, IntakePIDs.touronP, IntakePIDs.touronI, IntakePIDs.touronD, IntakePIDs.touronFF);

    // 
    // :> Shuffleboard PID Tuning
    //
    touronP = tab.add("TRN P Value:", touronP).getEntry();
    touronI = tab.add("TRN I Value:", touronI).getEntry();
    touronD = tab.add("TRN D Value:", touronD).getEntry();
      
    /* :> Sets the idlemode to break, 
      *   the reason why we do this is to make it so when the intake stops getting input it doesn't flail about
    */
    m_PivotMotor.setIdleMode(IdleMode.kBrake);
    
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
  protected void setPivotPIDFValues(SparkPIDController pidController, double p, double i, double d, double f) {
    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
    pidController.setFF(f);
  }
  /**
    * gets the current position of the Pivot
    * Currently only planned to be used for auto if used at all
    * ss I used it to tell if the Pivot is at the setpoint
    * @return Position of the Pivot
    * @author :>
    */
  public double getPivotMotorPosition() {
    return m_PivotAbsoluteEncoder.getPosition();
  }
  /**
   * Gets whether the Pivot is at the setPoint
   * @param position
   * @author ss
   */
  public boolean getPivotAtSetPoint(setPoints position) {
    return ((getPivotMotorPosition() / position.angle) > lowerBound) && ((getPivotMotorPosition() / position.angle) < upperBound);
  }
  /**
    * Sets the position of the Touron based off of an inputted setPoint
    * @param position
    * @author :>
    */
  public void setPositionReference(setPoints position) {
    m_PivotPID.setReference(position.angle, ControlType.kPosition);
  }

  public void turnOnIntake() {
    m_IntakePID.setReference(kV, CANSparkMax.ControlType.kVelocity);
  }

  public void turnOffIntake() {
    m_IntakePID.setReference(0, CANSparkMax.ControlType.kVelocity);
  }
}
