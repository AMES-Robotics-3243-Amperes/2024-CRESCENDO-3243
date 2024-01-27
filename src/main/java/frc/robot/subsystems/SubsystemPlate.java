// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
/*
 * Done, all notes are started with :>
 * Remove everything once you are done
 * No notes made in command
 */
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import static frc.robot.Constants.Plate.*;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.CommandMovePlateToPosition;
import frc.robot.utility.SubsystemBaseTestable;
import frc.robot.test.Test;
import frc.robot.test.TestUtil;

public class SubsystemPlate extends SubsystemBaseTestable {
  protected CANSparkMax motor;
  protected SparkAbsoluteEncoder encoder;
  protected SparkPIDController pidController;

  public ShuffleboardTab tab;

  protected GenericEntry pEntry;
  protected GenericEntry iEntry;
  protected GenericEntry dEntry;
  protected GenericEntry ffEntry;




  /** Creates a new SubsystemPlate. */
  public SubsystemPlate() {
    // Configure Shuffleboard
    tab = Shuffleboard.getTab("Plate");

    pEntry = tab.add("P", PIDValues.p).getEntry();
    iEntry = tab.add("I", PIDValues.i).getEntry();
    dEntry = tab.add("D", PIDValues.d).getEntry();
    ffEntry = tab.add("FF", PIDValues.ff).getEntry();

    // Initialize motor stuff
    motor = new CANSparkMax(motorID, MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    pidController = motor.getPIDController();

    // Configure PID
    pidController.setFeedbackDevice(encoder);
    pidController.setP(pEntry.getDouble(PIDValues.p));
    pidController.setI(iEntry.getDouble(PIDValues.i));
    pidController.setD(dEntry.getDouble(PIDValues.d));
    pidController.setFF(ffEntry.getDouble(PIDValues.ff));

    // Configure encoder
    encoder.setPositionConversionFactor(converstionFactor);
    encoder.setVelocityConversionFactor(converstionFactor);
  }



  /**
   * Tells the plate to go to the given position
   * @param position The position given as a double repersenting number of rotations,
   * where 0 is the fully folded up position.
   * @author H!
   */
  public void setPosition(double position) {
    pidController.setReference(position, ControlType.kPosition);
  }

  // H! This is awful, don't do this
  /**
   * Tells the plate to go to the given position
   * @param position The position given as a {@link Position} enum
   * @author H!
   */
  public void setPosition(Position position) {
    setPosition(position.position);
  }

  /**
   * Returns the current position of the plate as a number of rotations.
   * @author H!
   */
  public double getNumericPosition() {
    return encoder.getPosition();
  }
  /**
   * Returns the current position of the plate as a positional enum {@link Position}.
   * @return A {@link Position} repersenting the position, or null if not near any of them.
   * @author H!
   */
  public Position getDiscretePosition() {
    if (Math.abs(encoder.getVelocity()) < allowableVelDif) {
      if (Math.abs(encoder.getPosition() - Position.kStowed.position) < allowablePosDif) {
        return Position.kStowed;
      }
      else if (Math.abs(encoder.getPosition() - Position.kSpeaker.position) < allowablePosDif) {
        return Position.kSpeaker;
      }
      else if (Math.abs(encoder.getPosition() - Position.kAmp.position) < allowablePosDif) {
        return Position.kAmp;
      }
    }
    return null;
  }

  int timer = 0;
  @Override
  public void doPeriodic() {
    timer++;
    timer%=10;

    if (timer == 0) {
      pidController.setP(pEntry.getDouble(PIDValues.p));
      pidController.setI(iEntry.getDouble(PIDValues.i));
      pidController.setD(dEntry.getDouble(PIDValues.d));
      pidController.setFF(ffEntry.getDouble(PIDValues.ff));
    }
  }



  public static enum Position {
    kStowed  (Positions.stowed, "Stowed"),
    kAmp     (Positions.amp, "Amp"),
    kSpeaker (Positions.speaker, "Speaker");

    public final double position;
    public final String niceName;
    Position(double position, String niceName) {
      this.position = position;
      this.niceName = niceName;
    }

    public String toString() {
      return niceName;
    }
  }




  // ##### TESTS #####

  protected Test[] tests = new Test[]{
    new SetpointCommandTest(Position.kStowed), 
    new SetpointCommandTest(Position.kAmp), 
    new SetpointCommandTest(Position.kSpeaker)
  };
  @Override
  public Test[] getTests() {return tests;}

  public class SetpointCommandTest implements Test {
    public Position setpoint;
    public Command command;
    protected Timer timer = new Timer();
    protected Future<Boolean> atCorrectPosition = null;

    public SetpointCommandTest(Position setpoint) {
      this.setpoint = setpoint;
      command = new CommandMovePlateToPosition(SubsystemPlate.this, setpoint);
    }

    @Override public void testPeriodic() { TestUtil.assertBool(!timer.hasElapsed(5.), "Took too long"); }
    @Override public boolean testIsDone() {return !command.isScheduled();}
    @Override public void setupPeriodic() {CommandScheduler.getInstance().cancelAll(); command.schedule(); timer.restart();}
    @Override public boolean setupIsDone() {return true;}
    @Override public void closedownPeriodic() {  
      TestUtil.assertBool(SubsystemPlate.this.getDiscretePosition() == setpoint, "Command ended in the wrong place according to encoders");
      if (atCorrectPosition == null) {
        atCorrectPosition = TestUtil.askUserBool("Is the plate at the " + setpoint.niceName + " position?");
      }
    }
    @Override public boolean closedownIsDone() {
      if (atCorrectPosition.isDone()) {
        try {
          if (atCorrectPosition.get()) {
            return true;
          } else {
            throw new AssertionError("Command ended in the wrong place accodring to user");
          }
        } catch (InterruptedException | ExecutionException e) {
          throw new AssertionError("User query failed or was canceled");
        }
      } else {
        return false;
      }
    }
    @Override public String getName() { return "Setpoint Command Test: " + setpoint.niceName;}
    @Override public Test[] getDependencies() { return new Test[0];}
  }
}
