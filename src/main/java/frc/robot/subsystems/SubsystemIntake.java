// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.networktables.GenericEntry;
import frc.robot.test.Test;
import frc.robot.utility.SubsystemBaseTestable;

public class SubsystemIntake extends SubsystemBaseTestable {

  // ss Represents whether the Intake is on or off
  protected intakeState m_IntakeState = intakeState.Stopped;

  
  // 0? Creates IntakeMotor
  protected final CANSparkMax m_IntakeMotor;
  // 0? Creates IntakeMotor Relative Encoder. 
  protected final RelativeEncoder m_IntakeRelativeEncoder;
  // 0? Creates PIDController
  //protected final SparkPIDController m_IntakePID;

  //protected GenericEntry intakeP;
  //protected GenericEntry intakeI;
  //protected GenericEntry intakeD;
  //protected GenericEntry intakeFF;

  protected GenericEntry intakePos;
  protected GenericEntry intakeVel;
/*
  protected double intakePCurrent;
  protected double intakeICurrent;
  protected double intakeDCurrent;
  protected double intakeFFCurrent;
  protected double intakePPrevious;
  protected double intakeIPrevious;
  protected double intakeDPrevious;
  protected double intakeFFPrevious;
*/
  
  public enum intakeState {
    Intaking,
    Stopped,
    Outaking
  }

  // :> Creates the enum type to be able to pass in a setpoint from a command
  
  // :> Creates Shuffleboard tab to be able to put stuff on it relating to Intake
  protected ShuffleboardTab tab = Shuffleboard.getTab("Intake Tuning");


  /** Creates a new SubsystemIntake. */
  public SubsystemIntake() {
    // ss Sets up Intake Motor, Encoder, and PID
    m_IntakeMotor = new CANSparkMax(IntakeMotorID, CANSparkMax.MotorType.kBrushless);
    m_IntakeRelativeEncoder = m_IntakeMotor.getEncoder();
    m_IntakeRelativeEncoder.setVelocityConversionFactor(intakeConversionFactor);
    m_IntakeMotor.setSmartCurrentLimit(35);
    m_IntakeMotor.setSecondaryCurrentLimit(35);
    
    // 0? Sets up PID for Intake 
    //m_IntakePID.setP(IntakePIDs.kP);
    //m_IntakePID.setI(IntakePIDs.kI);
    //m_IntakePID.setD(IntakePIDs.kD);
    //m_IntakePID.setFF(IntakePIDs.kFF);
    // Sets the IntakeMotor to not run on startup
    //m_IntakePID.setReference(0, CANSparkMax.ControlType.kVelocity);

    //intakeP = tab.add("intake P Value:", IntakePIDs.kP).getEntry();
    //intakeI = tab.add("intake I Value:", IntakePIDs.kI).getEntry();
    //intakeD = tab.add("intake D Value:", IntakePIDs.kD).getEntry();
    //intakeFF = tab.add("intake FF Value:", IntakePIDs.kFF).getEntry();

    intakePos = tab.add("Intake Position:", m_IntakeRelativeEncoder.getPosition()).getEntry();
    intakeVel = tab.add("Intake Velocity:", m_IntakeRelativeEncoder.getVelocity()).getEntry();

    

  }

  @Override
  public void doPeriodic() {
    // This method will be called once per scheduler run

    intakePos.setDouble(m_IntakeRelativeEncoder.getPosition());
    intakeVel.setDouble(m_IntakeRelativeEncoder.getVelocity());

    //intakePCurrent = intakeP.getDouble(IntakePIDs.kP);
    //intakeICurrent = intakeI.getDouble(IntakePIDs.kI);
    //intakeDCurrent = intakeD.getDouble(IntakePIDs.kD);
    //intakeFFCurrent = intakeFF.getDouble(IntakePIDs.kFF);

    if (m_IntakeState == intakeState.Intaking) {
      //m_IntakePID.setReference(IntakePIDs.kV, ControlType.kVelocity);
      m_IntakeMotor.set(intakeFV);
    } else if (m_IntakeState == intakeState.Outaking) {
      m_IntakeMotor.set(intakeBV);
    } else {
      //m_IntakePID.setReference(0, ControlType.kVelocity);
      m_IntakeMotor.set(0.0);
    }
/*
    if (intakePPrevious != intakePCurrent) {
      m_IntakePID.setP(intakeP.getDouble(IntakePIDs.kP));
    }
    if (intakeIPrevious != intakeICurrent) {
      m_IntakePID.setI(intakeI.getDouble(IntakePIDs.kI));
    }
    if (intakeDPrevious != intakeDCurrent) {
      m_IntakePID.setD(intakeD.getDouble(IntakePIDs.kD));
    }
    if (intakeFFPrevious != intakeFFCurrent) {
      m_IntakePID.setFF(intakeFF.getDouble(IntakePIDs.kFF));
    }
*
    /*
    intakePPrevious = intakePCurrent;
    intakeIPrevious = intakeICurrent;
    intakeDPrevious = intakeDCurrent;
    intakeFFPrevious = intakeFFCurrent;
    */
  }

  /**
    * Sets the PIDValues of the intake? when called
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

  /**
   * makes the intake intake (velocity set in periodic) 
   * @author ss
   */
  public void intake() {
    m_IntakeState = intakeState.Intaking;
  }
  /**
   * Turns off the intake
   * @author ss
   */
  public void stop() {
    m_IntakeState = intakeState.Stopped;
  }
  /**
   * makes the intake outtake
   * @author ss
   */
  public void outtake() {
    m_IntakeState = intakeState.Outaking;
  }
  /**
   * If the intake is stopped, make the intake intake
   * if the intake is not stopped, stop it
   */
  public void toggleIntake() {
    if (m_IntakeState == intakeState.Stopped) {
      m_IntakeState = intakeState.Intaking;
    } else {
      m_IntakeState = intakeState.Stopped;
    }
  }



/*

  private PivotTest m_PivotTest = new PivotTest();
  private class PivotTest implements Test {

    @Override
    public void testPeriodic() {
      // Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'testPeriodic'");
    }

    @Override
    public boolean testIsDone() {
      // Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'testIsDone'");
    }

    @Override
    public void setupPeriodic() {
      // Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setupPeriodic'");
    }

    @Override
    public boolean setupIsDone() {
      // Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setupIsDone'");
    }

    @Override
    public void closedownPeriodic() {
      // Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'closedownPeriodic'");
    }

    @Override
    public boolean closedownIsDone() {
      // Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'closedownIsDone'");
    }

    @Override
    public String getName() {
      // Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'getName'");
    }

    @Override
    public Test[] getDependencies() {
      // Auto-generated method stub
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
    }

    @Override
    public boolean testIsDone() {
      // Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'testIsDone'");
    }

    @Override
    public void setupPeriodic() {
      timer.reset();

    }

    @Override
    public boolean setupIsDone() {
      // Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setupIsDone'");
    }

    @Override
    public void closedownPeriodic() {
      // Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'closedownPeriodic'");
    }

    @Override
    public boolean closedownIsDone() {
      // Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'closedownIsDone'");
    }

    @Override
    public String getName() {
      // Auto-generated method stub
      return "SampleTest";
    }

    @Override
    public Test[] getDependencies() {
      // Auto-generated method stub
      return new Test[]{
        m_PivotTest
      };
    }

  }
  */
  @Override
  public Test[] getTests() {
    // Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTests'");
  }
}
