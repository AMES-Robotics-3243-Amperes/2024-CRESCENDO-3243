// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import static frc.robot.Constants.IntakeConstants.*;

import java.util.concurrent.Future;

import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.networktables.GenericEntry;
import frc.robot.test.Test;
import frc.robot.test.TestUtil;
import frc.robot.utility.SubsystemBaseTestable;

public class SubsystemIntake extends SubsystemBaseTestable {

  // :> Creates the pivotMotor
  protected final CANSparkMax m_fourBarMotor = new CANSparkMax(fourBarMotor, MotorType.kBrushless);
  // :> Creates the pivot PIDController
  protected final SparkPIDController m_fourBarPID;
  // :> Creates the pivot AbsoluteEncoder
  protected final SparkAbsoluteEncoder m_fourBarAbsoluteEncoder; 
  // 0? Creates IntakeMotor
  protected final CANSparkMax m_IntakeMotor;
  // 0? Creates IntakeMotor Relative Encoder. 
  protected final RelativeEncoder m_IntakeRelativeEncoder;
  // 0? Creates PIDController
  protected final SparkPIDController m_IntakePID;

  // :> Limit Switches!
  protected final DigitalInput maxLimitSwitch = new DigitalInput(IntakeLimitSwitches.limitSwitchMax);
  protected final DigitalInput minLimitSwitch = new DigitalInput(IntakeLimitSwitches.limitSwitchMin);
  // :> Shuffleboard entries for us to be able to tune PIDs live
  protected GenericEntry fourBarP;
  protected GenericEntry fourBarI;
  protected GenericEntry fourBarD;
  protected GenericEntry fourBarFF;

  // :> Creates the enum type to be able to pass in a setpoint from a command
  public enum setPoints{
    position1 (fourBarSetPoint1),
    position2 (fourBarSetPoint2),
    position3 (fourBarSetPoint3);

    public final double angle;
    setPoints(double angle) {
      this.angle = angle;
    }
  }
  
  // :> Creates Shuffleboard tab to be able to put stuff on it relating to Intake
  protected ShuffleboardTab tab = Shuffleboard.getTab("Intake Tuning");


  /** Creates a new SubsystemIntake. */
  public SubsystemIntake() {
    // ss Sets up Intake Motor, Encoder, and PID
    m_IntakeMotor = new CANSparkMax(IntakeMotorID, CANSparkMax.MotorType.kBrushless);
    m_IntakeRelativeEncoder = m_IntakeMotor.getEncoder();
    m_IntakePID = m_IntakeMotor.getPIDController();
    m_IntakePID.setFeedbackDevice(m_IntakeRelativeEncoder);

    // 0? Sets up PID for Intake 
    m_IntakePID.setP(IntakePIDs.kP);
    m_IntakePID.setI(IntakePIDs.kI);
    m_IntakePID.setD(IntakePIDs.kD);
    m_IntakePID.setFF(IntakePIDs.kFF);
    // Sets the IntakeMotor to not run on startup
    m_IntakePID.setReference(0, CANSparkMax.ControlType.kVelocity);

    // :> Gets the absolute encoder from the motor
    m_fourBarAbsoluteEncoder = m_fourBarMotor.getAbsoluteEncoder(Type.kDutyCycle);
    // :> Gets the PIDController from the motor
    m_fourBarPID = m_fourBarMotor.getPIDController();
    // :> Accounts for the amount of turns it takes for the motor to actually move the intake
    m_fourBarAbsoluteEncoder.setPositionConversionFactor(fourBarConversionFactor);
    //:> Sets the PIDController to take in data from the absolute encoder when doing its calculations
    m_fourBarPID.setFeedbackDevice(m_fourBarAbsoluteEncoder);
   
    setPIDValues(m_fourBarPID, FourBarPIDs.fourBarP, FourBarPIDs.fourBarI, FourBarPIDs.fourBarD, FourBarPIDs.fourBarFF);

    // 
    // :> Shuffleboard PID Tuning
    //
   
    fourBarP = tab.add("FRBR P Value:", FourBarPIDs.fourBarP).getEntry();
    fourBarI = tab.add("FRBR I Value:", FourBarPIDs.fourBarI).getEntry();
    fourBarD = tab.add("FRBR D Value:", FourBarPIDs.fourBarD).getEntry();

    
      
    /* :> Sets the idlemode to break, 
      *   the reason why we do this is to make it so when the intake stops getting input it doesn't flail about
    */
    m_fourBarMotor.setIdleMode(IdleMode.kBrake);
    
  }

  @Override
  public void doPeriodic() {
    // This method will be called once per scheduler run
    // 0? Sets PID Values, updates when changed. 
    m_fourBarPID.setP(fourBarP.getDouble(FourBarPIDs.fourBarP));
    m_fourBarPID.setI(fourBarI.getDouble(FourBarPIDs.fourBarI));
    m_fourBarPID.setD(fourBarD.getDouble(FourBarPIDs.fourBarD));

    // :> Bryce said this is the best way to do it theoretically, might be wrong. We'll find out ¯\_(ツ)_/¯
    if (maxLimitSwitch.get()) {
      m_fourBarAbsoluteEncoder.setZeroOffset((getFourBarMotorPosition() - (fourBarSetPoint3 - fourBarSetPoint1)));
    }
    if (minLimitSwitch.get()) {
      m_fourBarAbsoluteEncoder.setZeroOffset(getFourBarMotorPosition());
    }
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
   * Gets whether the fourBar is at the setPoint
   * @param position
   * @author ss
   */
  public boolean getFourBarAtSetPoint(setPoints position) {
    return ((getFourBarMotorPosition() / position.angle) > lowerBound) && ((getFourBarMotorPosition() / position.angle) < upperBound);
  }
  /**
    * Sets the position of the fourBar based off of an inputted setPoint
    * @param position
    * @author :>
    */
  public void setFourBarPositionReference(setPoints position) {
    m_fourBarPID.setReference(position.angle, ControlType.kPosition);
  }
  // 0? Sets velocity of Intake when turned on
  public void turnOnIntake() {
    m_IntakePID.setReference(IntakePIDs.kV, CANSparkMax.ControlType.kVelocity);
  }
  // 0? Sets velocity of Intake when turned off
  public void turnOffIntake() {
    m_IntakePID.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

  private PivotTest m_PivotTest = new PivotTest();
  private class PivotTest implements Test {

    @Override
    public void testPeriodic() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'testPeriodic'");
    }

    @Override
    public boolean testIsDone() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'testIsDone'");
    }

    @Override
    public void setupPeriodic() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setupPeriodic'");
    }

    @Override
    public boolean setupIsDone() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setupIsDone'");
    }

    @Override
    public void closedownPeriodic() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'closedownPeriodic'");
    }

    @Override
    public boolean closedownIsDone() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'closedownIsDone'");
    }

    @Override
    public String getName() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'getName'");
    }

    @Override
    public Test[] getDependencies() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'getDependencies'");
    }
    
  }

  private IntakeTest m_IntakeTest = new IntakeTest();
  private class IntakeTest implements Test {

    Timer timer = new Timer();
    Future<Boolean> response = null;
    
    @Override
    public void testPeriodic() {
      // ss confirm from user that everything worked.
    if(response == null){
      response = TestUtil.askUserBool("Did the Intake Run for ~5 Seconds");
    }
      // ss turn on the intake
  
      // ss wait 5 seconds
      if(timer.hasElapsed(5)){
        turnOffIntake();
      }
      else  {turnOnIntake();}
      
      // ss turn off the intake
     
    }

    @Override
    public boolean testIsDone() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'testIsDone'");
    }

    @Override
    public void setupPeriodic() {
      timer.reset();

    }

    @Override
    public boolean setupIsDone() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setupIsDone'");
    }

    @Override
    public void closedownPeriodic() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'closedownPeriodic'");
    }

    @Override
    public boolean closedownIsDone() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'closedownIsDone'");
    }

    @Override
    public String getName() {
      // TODO Auto-generated method stub
      return "SampleTest";
    }

    @Override
    public Test[] getDependencies() {
      // TODO Auto-generated method stub
      return new Test[]{
        m_PivotTest
      };
    }

  }

  @Override
  public Test[] getTests() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTests'");
  }
}
