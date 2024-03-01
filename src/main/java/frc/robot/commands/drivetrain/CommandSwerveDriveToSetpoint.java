// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.Optional;
import java.util.function.Supplier;

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
  Supplier<Pose2d> goal;
  Optional<Pose2d> currentGoal = Optional.empty();

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
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.goal = () -> goal;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrainSubsystem);
  }

  /** Creates a new CommandSwerveDriveToSetpoint. */
  public CommandSwerveDriveToSetpoint(SubsystemSwerveDrivetrain drivetrainSubsystem, Supplier<Pose2d> goal) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.goal = goal;

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

    currentGoal = Optional.of(poseAngleModulus(goal.get()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentRobotPose = poseAngleModulus(DataManager.currentRobotPose.get().toPose2d());
    double xSpeed = xPidController.calculate(currentRobotPose.getTranslation().getX(), currentGoal.get().getX());
    double ySpeed = yPidController.calculate(currentRobotPose.getTranslation().getY(), currentGoal.get().getY());
    Translation2d speeds = new Translation2d(xSpeed, ySpeed);

    speeds = speeds.rotateBy(currentRobotPose.getRotation().times(-1));

    Translation2d offset = currentGoal.get().getTranslation().minus(currentRobotPose.getTranslation());
    Translation2d correctOffset = offset.times(AutoConstants.kSetpointMinSpeed / offset.getNorm());

    if (offset.getNorm() != 0 && !isAtCorrectSpot(false)) {
      speeds = speeds.plus(correctOffset);
    }

    // :3 get rotation speed
    double rotationSpeed = thetaPidController.calculate(MathUtil.angleModulus(currentRobotPose.getRotation().getRadians()),
      currentGoal.get().getRotation().getRadians());

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
    if (!interrupted) {System.out.println("\nDriving Done\n");}
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAtCorrectSpot(true);
  }

  // Returns true when the robot is at the correct spot
  public boolean isAtCorrectSpot(boolean considerRotation) {
    Pose2d currentRobotPose = DataManager.currentRobotPose.get().toPose2d();
    boolean positionCorrect =
      currentRobotPose.getTranslation().getDistance(currentGoal.get().getTranslation()) < AutoConstants.kMaxSetpointDistance;

    return positionCorrect && (getRotationCorrect() || !considerRotation);
  }

  /**
   * Helper function for getting if rotation is correct
   * 
   * @author :3
   */
  private boolean getRotationCorrect() {
    double currentRotationRadians = poseAngleModulus(DataManager.currentRobotPose.get().toPose2d()).getRotation().getRadians();
    double goalRotationRadians = goal.get().getRotation().getRadians();
    return MathUtil.isNear(currentRotationRadians, goalRotationRadians, AutoConstants.kMaxSetpointRotationError.getRadians())
      || MathUtil.isNear(currentRotationRadians, goalRotationRadians + (Math.PI * 2), AutoConstants.kMaxSetpointRotationError.getRadians())
      || MathUtil.isNear(currentRotationRadians, goalRotationRadians, AutoConstants.kMaxSetpointRotationError.getRadians() + (Math.PI * 2));
  }

  /** Helper function that performs angle modulus on a Pose2d;
   * 
   * @author :3
   */
  private Pose2d poseAngleModulus(Pose2d pose) {
    Rotation2d fixedAngle = Rotation2d.fromRadians(MathUtil.angleModulus(pose.getRotation().getRadians()));
    return new Pose2d(pose.getX(), pose.getY(), fixedAngle);
  }
}
