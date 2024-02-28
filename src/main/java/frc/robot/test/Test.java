// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

/** Add your docs here. */
public interface Test {
    public void testPeriodic();

    public boolean testIsDone();

    public void setupPeriodic();

    public boolean setupIsDone();

    public void closedownPeriodic();

    public boolean closedownIsDone();

    public String getName();

    /** 
     * <p><b>This method must always return the same thing, down to the refrences. That means
     * this method CANNOT create new test objects each time it is run.</b></p>
     * 
     * <p>This should return a list of tests that must run before this one. By 
     * default, it is assumed all tests must succeed before this test should run.
     * If you wish to change this behavior, see {@link Test#getDependencySuccessRequirements()}.</p>
     * 
     */
    public Test[] getDependencies();

    /**
     * Should return an array corresponding to whether each test listed returned by {@link Test#getDependencies()}
     * needs to succeed (true) of fail (false), for this test to run.
     */
    default public boolean[] getDependencySuccessRequirements() {
        boolean[] out = new boolean[getDependencies().length];
        for (int i = 0; i < out.length; i++) {
            out[i] = true;
        }
        return out;
    }
}
