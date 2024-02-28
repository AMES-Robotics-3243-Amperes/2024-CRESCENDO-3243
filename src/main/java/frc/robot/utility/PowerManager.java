package frc.robot.utility;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class PowerManager {

    public static double frontRightDrivetrainMotorPresentCurrent;
    public static double frontLeftDrivetrainMotorPresentCurrent;
    public static double rearRightDrivetrainMotorPresentCurrent;
    public static double rearLeftDrivetrainMotorPresentCurrent;

    /** 
     * @return Gets the Voltage Drop of the entire robot using how much lower it is from 12 Volts
     * 
     * @author :>
    */
    public static double getCurrentRobotVoltageDrop() {
        double currentVoltageDrop = RobotController.getBatteryVoltage();
    if (12 - currentVoltageDrop < 0) {
        return 0;
    } else {
    return currentVoltageDrop;
    }
    }
    
    // :> This method gets how much current the robot is pulling from taking its voltage and using our resistance constant
    /**
     * @return All of the current the robot is drawing
     * 
     * @author :>
     */
    public static final double getTotalRobotPowerDraw() {

        // :> Gets the robot's current based off of Ohm's Law (V=IR)
        double robotCurrent = (getCurrentRobotVoltageDrop() / Constants.PowerManager.robotResistance);
  
        return robotCurrent;
    }
    // :> No idea why we would use this but if it comes up it comes up
    public static final double getDriveTrainPowerDraw() {
        double drivetrainCurrent = (frontRightDrivetrainMotorPresentCurrent + frontLeftDrivetrainMotorPresentCurrent + rearLeftDrivetrainMotorPresentCurrent + rearRightDrivetrainMotorPresentCurrent);

        return drivetrainCurrent;
    }
    /*  :> This method will return a driveSpeedDamper for which can be determined by how much current we are pulling.
        *  This method has a nearly redundant amount of failsafes to stop us from pulling to much amperage and browning out
        *  The speed dampers it is returning are going to be based on its current current
        *  It is really important that we make fail safes throughout the entire robots firing motors that we don't go over 200 amps.
        *  Luckily all of those potential issues can be solved with power manager in similiar methods that limit current output
    */

    /**
     * 
     * @return The top speed the robot should reach
     * @author :>
     */
    public static final double getDriveSpeedDamper() {
    
        
        if (RobotController.getBatteryVoltage() < Constants.PowerManager.softVoltageCap) {
        return Constants.DriveTrain.DriveConstants.kSlowDrivingSpeedDamper;
        } else {
            return Constants.DriveTrain.DriveConstants.kDrivingSpeedDamper;
        }
    }
    /* :> We use a function of Acceleration Vs. Voltage Drop to determine
     * how much it should slow down the acceleration (jerk)
     * This can be seen with this graph: https://www.desmos.com/calculator/5uecr6ja6o
     * The x intercept is at when the Acceleration is set to 0
     * The tick after the acceleration crosses 8.1v it will slow down acceleration to 0 if the
     * voltage drop goes above 4.5 Volts (7.5 Total Volts)
    */

    /**
     * 
     * @return The speed at which the robot should accelerate at
     * @author :>
     */
    public static final double getDriveAccelerationDampener() {
        if (getCurrentRobotVoltageDrop() > 4.5) {
            return 1;
        } else if (RobotController.getBatteryVoltage() < Constants.PowerManager.softVoltageCap) {
            return ((2.6 * (getCurrentRobotVoltageDrop())) - (26 * (getCurrentRobotVoltageDrop() - 3.9)) );
        } else {
            return Constants.DriveTrain.DriveConstants.kMaxDrivingAcceleration;
        }
    }
}
