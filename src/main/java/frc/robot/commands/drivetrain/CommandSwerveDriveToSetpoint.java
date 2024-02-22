// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveTrain.DriveConstants.AutoConstants;
import frc.robot.Constants.DriveTrain.DriveConstants.ChassisKinematics;
import frc.robot.DataManager;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

public class CommandSwerveDriveToSetpoint extends Command {
  SubsystemSwerveDrivetrain drivetrainSubsystem;
  Pose2d goal;

  ProfiledPIDController xPidController = new ProfiledPIDController(AutoConstants.kSetpointP,
    AutoConstants.kSetpointI,
    AutoConstants.kSetpointD,
    new TrapezoidProfile.Constraints(AutoConstants.kMaxDrivingVelocity,
      AutoConstants.kMaxDrivingAcceleration));

  ProfiledPIDController yPidController = new ProfiledPIDController(AutoConstants.kSetpointP,
    AutoConstants.kSetpointI,
    AutoConstants.kSetpointD,
    new TrapezoidProfile.Constraints(AutoConstants.kMaxDrivingVelocity,
      AutoConstants.kMaxDrivingAcceleration));

  ProfiledPIDController thetaPidController = new ProfiledPIDController(AutoConstants.kTuringP,
    AutoConstants.kTurningI,
    AutoConstants.kTurningD,
      new TrapezoidProfile.Constraints(AutoConstants.kMaxAngularVelocityRadians,
        AutoConstants.kMaxAngularAccelerationRadians));

  /** Creates a new CommandSwerveDriveToSetpoint. */
  public CommandSwerveDriveToSetpoint(SubsystemSwerveDrivetrain drivetrainSubsystem, Pose2d goal) {
    Rotation2d fixedRotation = Rotation2d.fromRadians(MathUtil.angleModulus(goal.getRotation().getRadians()));
    Pose2d fixedGoal = new Pose2d(goal.getTranslation(), fixedRotation);

    this.drivetrainSubsystem = drivetrainSubsystem;
    this.goal = fixedGoal;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentRobotPose = DataManager.currentRobotPose.get().toPose2d();

    xPidController.reset(currentRobotPose.getTranslation().getX());
    yPidController.reset(currentRobotPose.getTranslation().getY());
    thetaPidController.reset(currentRobotPose.getRotation().getRadians());
    thetaPidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentRobotPose = DataManager.currentRobotPose.get().toPose2d();
    double xSpeed = xPidController.calculate(currentRobotPose.getTranslation().getX(), goal.getX());
    double ySpeed = yPidController.calculate(currentRobotPose.getTranslation().getY(), goal.getY());
    Translation2d speeds = new Translation2d(xSpeed, ySpeed);

    speeds = speeds.rotateBy(DataManager.currentRobotPose.get().toPose2d().getRotation().times(-1));

    // :3 get rotation speed
    double rotationSpeed = thetaPidController.calculate(MathUtil.angleModulus(currentRobotPose.getRotation().getRadians()),
      goal.getRotation().getRadians());

    

    // :3 drive with those speeds
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speeds.getX(), speeds.getY(), rotationSpeed);
    SwerveModuleState[] moduleStates = ChassisKinematics.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    drivetrainSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    SwerveModuleState[] moduleStates = ChassisKinematics.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    drivetrainSubsystem.setModuleStates(moduleStates);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
