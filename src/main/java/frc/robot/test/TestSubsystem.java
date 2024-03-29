// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.SubsystemBaseTestable;

public class TestSubsystem extends SubsystemBaseTestable {

  public double a = 1.3;

  /** Creates a new TestSubsystem. */
  public TestSubsystem() {}

  @Override
  public void doPeriodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public String getName() {
    return "Test Subsystem";
  }

  public class ExampleTest1 implements Test {

    double timer = 0;

    @Override
    public void testPeriodic() {
      timer++;
      a++;
      assert a == timer;
    }

    @Override
    public boolean testIsDone() {
      return timer == 10;
    }

    @Override
    public void setupPeriodic() {
      a = 0;
      timer = 0;
    }

    @Override
    public boolean setupIsDone() {
      return true;
    }

    @Override
    public void closedownPeriodic() {}

    @Override
    public boolean closedownIsDone() {
      return true;
    }

    @Override
    public String getName() {
      return "ExampleTest1";
    }

    @Override
    public Test[] getDependencies() {
      // Auto-generated method stub
      return null; // add dependencies and such
    }
    
  }

  public class ExampleTest2 implements Test {

    @Override
    public void testPeriodic() {
      // Auto-generated method stub
      
    }

    @Override
    public boolean testIsDone() {
      // Auto-generated method stub
      return false;
    }

    @Override
    public void setupPeriodic() {
      // Auto-generated method stub
      
    }

    @Override
    public boolean setupIsDone() {
      // Auto-generated method stub
      return false;
    }

    @Override
    public void closedownPeriodic() {
      // Auto-generated method stub
      
    }

    @Override
    public boolean closedownIsDone() {
      // Auto-generated method stub
      return false;
    }

    @Override
    public String getName() {
      // Auto-generated method stub
      return null;
    }

    @Override
    public Test[] getDependencies() {
      // Auto-generated method stub
      return null;
    }
    
  }

  @Override
  public Test[] getTests() {
    return new Test[] {
      new ExampleTest1()
    };
  }
}
