// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.FourBarPIDs;
import frc.robot.Constants.IntakeConstants.IntakeLimitSwitches;

public class SubsystemFourBar extends SubsystemBase {
  // :> Creates the pivotMotor
  protected final CANSparkMax m_fourBarMotor = new CANSparkMax(fourBarMotor, MotorType.kBrushless);
  // :> Creates the pivot PIDController
  protected final SparkPIDController m_fourBarPID;
  // :> Creates the pivot AbsoluteEncoder
  protected final SparkAbsoluteEncoder m_fourBarAbsoluteEncoder; 
  // :> Limit Switches!
  protected final DigitalInput maxLimitSwitch = new DigitalInput(IntakeLimitSwitches.limitSwitchMax);
  protected final DigitalInput minLimitSwitch = new DigitalInput(IntakeLimitSwitches.limitSwitchMin);

  // :> Shuffleboard entries for us to be able to tune PIDs live
  protected GenericEntry fourBarP;
  protected GenericEntry fourBarI;
  protected GenericEntry fourBarD;
  protected GenericEntry fourBarFF;
  protected double fourBarPCurrentState;
  protected double fourBarPPreviosuState;
  protected double fourBarICurrentState;
  protected double fourBarIPreviosuState;
  protected double fourBarDCurrentState;
  protected double fourBarDPreviosuState;
  protected double fourBarFCurrentState;
  protected double fourBarFPreviosuState;


  // :> Creates the enum type to be able to pass in a setpoint from a command
  public enum SetPoints{
    fourBarNotDeployedPosition (fourBarUndeployedSetPoint, -0.02),// rename please? :point_right: :point_left: :pleading:
    fourBarHalfDeployedPosition (fourBarHalfDeployedSetPoint), // Done, *blushes*
    fourBarFullyDeployedPosition (fourBarFullyDeployedSetPoint, 0.02);

    public final double angle;
    public final double pushOffset;

    SetPoints(double angle, double pushOffset) {
      this.angle = angle;
      this.pushOffset = pushOffset;
    }

    SetPoints(double angle) {
      this(angle, 0);
    }
  }
  
  
  // :> Creates Shuffleboard tab to be able to put stuff on it relating to fourBar
  /** Creates a new SubsystemFourBar. */
  public SubsystemFourBar() {


    m_fourBarMotor.setSmartCurrentLimit(40);

    m_fourBarMotor.setIdleMode(IdleMode.kBrake);


    // :> Gets the absolute encoder from the motor
    m_fourBarAbsoluteEncoder = m_fourBarMotor.getAbsoluteEncoder(Type.kDutyCycle);
    // :> Gets the PIDController from the motor
    m_fourBarPID = m_fourBarMotor.getPIDController();
    // :> Accounts for the amount of turns it takes for the motor to actually move the fourBar
    m_fourBarAbsoluteEncoder.setPositionConversionFactor(fourBarConversionFactor);
    //:> Sets the PIDController to take in data from the absolute encoder when doing its calculations
    m_fourBarPID.setFeedbackDevice(m_fourBarAbsoluteEncoder);

    m_fourBarAbsoluteEncoder.setZeroOffset(.4292704 - Constants.IntakeConstants.fourBarUndeployedSetPoint);
   
    setPIDValues(m_fourBarPID, FourBarPIDs.fourBarP, FourBarPIDs.fourBarI, FourBarPIDs.fourBarD, FourBarPIDs.fourBarFF);


    // 
    // :> Shuffleboard PID Tuning
    //
   

    /* :> Sets the idlemode to break, 
      *   the reason why we do this is to make it so when the fourBar stops getting input it doesn't flail about
    */
    m_fourBarMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // && Widget for the tauron position on shuffleboard
    SmartDashboard.putNumber("4BarRotations",m_fourBarAbsoluteEncoder.getPosition());

    // :> Sets the current state of the shuffleboard inputs
    //fourBarPCurrentState = fourBarP.getDouble(FourBarPIDs.fourBarP);
    //fourBarICurrentState = fourBarI.getDouble(FourBarPIDs.fourBarI);
    //fourBarDCurrentState = fourBarD.getDouble(FourBarPIDs.fourBarD);
    //fourBarFCurrentState = fourBarFF.getDouble(FourBarPIDs.fourBarFF);
    
    // :> Bryce said this is the best way to do it theoretically, might be wrong. We'll find out ¯\_(ツ)_/¯
    /*
    if (maxLimitSwitch.get()) {
      m_fourBarAbsoluteEncoder.setZeroOffset(((getFourBarMotorPosition() + m_fourBarAbsoluteEncoder.getZeroOffset()) - (fourBarFullyDeployedSetPoint - fourBarUndeployedSetPoint)));
    }
    if (minLimitSwitch.get()) {
      m_fourBarAbsoluteEncoder.setZeroOffset(m_fourBarAbsoluteEncoder.getZeroOffset() + getFourBarMotorPosition());
    }
    */
    /* 
    if (fourBarPPreviosuState != fourBarPCurrentState) {
      m_fourBarPID.setP(fourBarP.getDouble(FourBarPIDs.fourBarP));
    }
    if (fourBarIPreviosuState != fourBarICurrentState) {
      m_fourBarPID.setI(fourBarI.getDouble(FourBarPIDs.fourBarI));
    }
    if (fourBarDPreviosuState != fourBarDCurrentState) {
      m_fourBarPID.setD(fourBarD.getDouble(FourBarPIDs.fourBarD));
    }
    if (fourBarFPreviosuState != fourBarFCurrentState) {
      m_fourBarPID.setFF(fourBarFF.getDouble(FourBarPIDs.fourBarFF));
    }

    fourBarPPreviosuState = fourBarPCurrentState;
    fourBarIPreviosuState = fourBarICurrentState;
    fourBarDPreviosuState = fourBarDCurrentState;
    fourBarFPreviosuState = fourBarFCurrentState;
    */
  }
  /**
    * Sets the PIDValues of the fourBar when called
    * @param pidController
    * @param p
    * @param i
    * @param d
    * @param f
    * @author :>
    */
    protected void setPIDValues(SparkPIDController pidController, double p, double i, double d, double f) {
      pidController.setP(p);
      pidController.setI(i);
      pidController.setD(d);
      pidController.setFF(f);
    }
    /**
      * gets the current position of the fourBar
      * Currently only planned to be used for auto if used at all
      * ss I used it to tell if the fourBar is at the setpoint
      * @return Position of the fourBar
      * @author :>
      */
    public double getFourBarMotorPosition() {
      return m_fourBarAbsoluteEncoder.getPosition();
    }
    /**
    * Sets the position of the fourBar based off of an inputted setPoint
    * @param position
    * @author :>
    */
    public void setFourBarPositionReference(SetPoints position) {
      m_fourBarPID.setReference(position.angle + position.pushOffset, ControlType.kPosition);
    }
    /**
    * Gets whether the fourBar is at the setPoint
    * @param position
    * @author ss
    */
    public boolean getFourBarAtSetPoint(SetPoints position) {
      return Math.abs(position.angle - getFourBarMotorPosition()) < allowableDifference;
    }
    
}
