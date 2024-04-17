// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.Constants.DriveTrain.DriveConstants.ChassisKinematics;
import frc.robot.DataManager;
import frc.robot.JoyUtil;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

public class CommandSwerveTeleopDrive extends Command {

  // :3 subsystem
  private final SubsystemSwerveDrivetrain m_SubsystemSwerveDrivetrain;

  // :3 driver joyutil
  private final JoyUtil m_controller;

  // :3 teleop driving should be reversed depending on field side
  private boolean reverse = false;

  // :3 if driving is field relative
  private boolean fieldRelative = true;

  // :3 if the presses to toggle things have been handled yet
  private boolean hasHandledFieldRelativeToggle = false;
  private boolean hasHandledPhotonVisionToggle = false;

  /**
   * Creates a new SwerveTeleopCommand.
   * 
   * @author :3
   */
  public CommandSwerveTeleopDrive(SubsystemSwerveDrivetrain subsystem, JoyUtil controller) {
    m_SubsystemSwerveDrivetrain = subsystem;
    m_controller = controller;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // :3 handle field relativity
    if (m_controller.getBack() && !hasHandledFieldRelativeToggle) {
      fieldRelative = !fieldRelative;
      hasHandledFieldRelativeToggle = true;
    } else if (!m_controller.getBack()) {
      hasHandledFieldRelativeToggle = false;
    }

    // :3 handle photon vision
    if (m_controller.getStart() && !hasHandledPhotonVisionToggle) {
      DataManager.currentRobotPose.setPhotonVision(!DataManager.currentRobotPose.getPhotonVisionStatus());
      hasHandledPhotonVisionToggle = true;
    } else if (!m_controller.getStart()) {
      hasHandledPhotonVisionToggle = false;
    }

    // :3 get x and y speeds
    double xSpeed = -m_controller.getLeftY() * 0.5 * (reverse ? -1 : 1) * DataManager.currentVelocityConstant.get();
    double ySpeed = -m_controller.getLeftX() * 0.5 * (reverse ? -1 : 1) * DataManager.currentVelocityConstant.get();
    Translation2d speeds = new Translation2d(xSpeed, ySpeed);

    if (fieldRelative) {
      speeds = speeds.rotateBy(DataManager.currentRobotPose.get().toPose2d().getRotation().times(-1));
    }

    // :3 get rotation speed
    double controllerRightX = m_controller.getRightX();
    double rotationSpeed = -controllerRightX * DriveConstants.kAngularSpeedDamper;

    // :3 drive with those speeds
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speeds.getX(), speeds.getY(), rotationSpeed);
    SwerveModuleState[] moduleStates = ChassisKinematics.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    m_SubsystemSwerveDrivetrain.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    SwerveModuleState[] moduleStates = ChassisKinematics.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    m_SubsystemSwerveDrivetrain.setModuleStates(moduleStates);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Sets if driving should be reversed or not
   * 
   * @param value if driving should be reversed
   * 
   * @author :3
   */
  public void setReverse(boolean value) {
    reverse = value;
  }
}