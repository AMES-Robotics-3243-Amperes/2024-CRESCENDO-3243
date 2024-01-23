// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

import java.util.concurrent.Future;

import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.networktables.GenericEntry;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.IntakePIDs;
import frc.robot.test.Test;
import frc.robot.test.TestUtil;
import frc.robot.utility.SubsystemBaseTestable;

public class SubsystemIntake extends SubsystemBaseTestable {

  // :> Creates the pivotMotor
  protected final CANSparkMax m_PivotMotor = new CANSparkMax(Constants.IntakeConstants.touronMotor, MotorType.kBrushless);
  // :> Creates the pivot PIDController
  protected final SparkPIDController m_PivotPID;
  // :> Creates the pivot AbsoluteEncoder
  protected final SparkAbsoluteEncoder m_PivotAbsoluteEncoder; 
  // 0? Creates IntakeMotor
  protected final CANSparkMax m_IntakeMotor;
  // 0? Creates IntakeMotor Relative Encoder. 
  protected final RelativeEncoder m_IntakeRelativeEncoder;
  // 0? Creates PIDController
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
    // ss Sets up Intake Motor, Encoder, and PID
    m_IntakeMotor = new CANSparkMax(IntakeMotorID, CANSparkMax.MotorType.kBrushless);
    m_IntakeRelativeEncoder = m_IntakeMotor.getEncoder();
    m_IntakePID = m_IntakeMotor.getPIDController();
    m_IntakePID.setFeedbackDevice(m_IntakeRelativeEncoder);

    // 0? Sets up PID for Intake 
    m_IntakePID.setP(kP);
    m_IntakePID.setI(kI);
    m_IntakePID.setD(kD);
    m_IntakePID.setFF(kFF);
    // Sets the IntakeMotor to not run on startup
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
  public void doPeriodic() {
    // This method will be called once per scheduler run
  }

  /**
    * Sets the PIDValues of the Pivot when called
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
    * Sets the position of the Pivot based off of an inputted setPoint
    * @param position
    * @author :>
    */
  public void setPositionReference(setPoints position) {
    m_PivotPID.setReference(position.angle, ControlType.kPosition);
  }
  // 0? Sets velocity of Intake when turned on
  public void turnOnIntake() {
    m_IntakePID.setReference(kV, CANSparkMax.ControlType.kVelocity);
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

    Timer timer = new Timer()
    Future<Boolean> response;

    @Override
    public void testPeriodic() {
      // ss confirm from user that everything worked.
      response = TestUtil.askUserBool("Did the Intake Run for ~5 Seconds");
      // ss turn on the intake
      turnOnIntake();
      // ss wait 5 seconds
      timer.wait(5000);
      // ss turn off the intake
      turnOffIntake();
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
