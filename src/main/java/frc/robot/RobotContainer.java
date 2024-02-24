// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.JoyUtilConstants;
import frc.robot.Constants.DriveTrain.DriveConstants.AutoConstants;
import frc.robot.commands.automatics.CommandDriveAndIntake;
import frc.robot.commands.automatics.CommandPickup;
import frc.robot.commands.automatics.CommandPickupFieldNote;
import frc.robot.commands.automatics.CommandScoreInAmp;
import frc.robot.commands.automatics.CommandScoreInSpeakerRight;
import frc.robot.commands.automatics.CommandScoreInSpeaker;
import frc.robot.commands.automatics.CommandScoreInSpeakerLeft;
import frc.robot.commands.climber.CommandClimberTeleop;
import frc.robot.commands.drivetrain.CommandSwerveDriveToSetpoint;
import frc.robot.commands.drivetrain.CommandSwerveTeleopDrive;
import frc.robot.commands.intake.CommandFourBarMoveFourBar;
import frc.robot.commands.intake.CommandFourBarTeleop;
import frc.robot.commands.intake.CommandIntakeTeleop;
import frc.robot.commands.shooter.CommandShooterStopInstant;
import frc.robot.commands.shooter.CommandShooterTeleopAmp;
import frc.robot.commands.shooter.CommandShooterTeleopSpeaker;
import frc.robot.subsystems.SubsystemPhotonvision;
import frc.robot.subsystems.SubsystemShooter;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.subsystems.SubsystemFourBar.SetPoints;
import frc.robot.utility.SpeakerPosition;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemClimber;
import frc.robot.subsystems.SubsystemFourBar;

import java.io.IOException;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //
  // :3 Controller
  //

  private final JoyUtil primaryController = new JoyUtil(JoyUtilConstants.primaryControllerID);
  private final JoyUtil secondaryController = new JoyUtil(JoyUtilConstants.secondaryControllerID);
  
  //
  // :3 SUBSYSTEMS
  //
  
  private final SubsystemSwerveDrivetrain m_SubsystemSwerveDrivetrain = new SubsystemSwerveDrivetrain();
  private final SubsystemIntake m_subsystemIntake = new SubsystemIntake();
  private final SubsystemShooter m_SubsystemShooter = new SubsystemShooter();
  private final SubsystemClimber m_SubsystemClimber = new SubsystemClimber();
  private final SubsystemFourBar m_SubsystemFourBar = new SubsystemFourBar();
  //private final SubsystemPlate m_subsystemPlate = new SubsystemPlate();

  //
  // :3 COMMANDS
  //


  private final CommandSwerveTeleopDrive m_CommandSwerveTeleopDrive = new CommandSwerveTeleopDrive(m_SubsystemSwerveDrivetrain, primaryController);
  private final CommandIntakeTeleop m_teleopCommandIntake = new CommandIntakeTeleop(m_subsystemIntake, secondaryController);
  private final CommandFourBarTeleop m_CommandFourBarTeleop = new CommandFourBarTeleop(m_SubsystemFourBar, secondaryController);
  private final CommandShooterTeleopAmp m_CommandShooterTeleopAmp = new CommandShooterTeleopAmp(m_SubsystemShooter);
  private final CommandShooterTeleopSpeaker m_CommandShooterTeleopSpeaker = new CommandShooterTeleopSpeaker(m_SubsystemShooter);
  //private final CommandShooterStop m_CommandShooterStop = new CommandShooterStop(m_SubsystemShooter);

  private final CommandClimberTeleop m_CommandClimberTeleop = new CommandClimberTeleop(m_SubsystemClimber, primaryController);
  //private final CommandPlateTeleop m_commandPlateTeleop = new CommandPlateTeleop(m_subsystemPlate, secondaryController);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_SubsystemSwerveDrivetrain.setDefaultCommand(m_CommandSwerveTeleopDrive);
    m_subsystemIntake.setDefaultCommand(m_teleopCommandIntake);
    m_SubsystemClimber.setDefaultCommand(m_CommandClimberTeleop);
    m_SubsystemFourBar.setDefaultCommand(m_CommandFourBarTeleop);
    //m_subsystemPlate.setDefaultCommand(m_commandPlateTeleop);
     
    try {
      new SubsystemPhotonvision();
    } catch (IOException e) {
      System.out.println("ruh roh, camera moment");
    }
    
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // ss Intake
    //secondaryController.a().whileTrue(new CommandIntakeUntilSensed(m_subsystemIntake));
    // ss Outtake
    //secondaryController.b().whileTrue(new CommandOuttakeUntilSensed(m_subsystemIntake));
    // && Toggle amp shooting
    secondaryController.x().toggleOnTrue(m_CommandShooterTeleopAmp);
    // && toggle speaker shooting
    secondaryController.y().toggleOnTrue(m_CommandShooterTeleopSpeaker);
    secondaryController.rightBumper().whileTrue(new CommandPickup(m_subsystemIntake, m_SubsystemFourBar));

    secondaryController.rightBumper().onFalse(new CommandFourBarMoveFourBar(m_SubsystemFourBar, SetPoints.fourBarNotDeployedPosition));

    primaryController.a().whileTrue(new CommandScoreInAmp(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemShooter, m_SubsystemFourBar));
    
    primaryController.b().whileTrue(new CommandScoreInSpeakerRight(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemShooter, m_SubsystemFourBar));
    primaryController.y().whileTrue(new CommandScoreInSpeaker(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemShooter, m_SubsystemFourBar, SpeakerPosition.center));
    primaryController.x().whileTrue(new CommandScoreInSpeakerLeft(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemShooter, m_SubsystemFourBar));

    primaryController.b().onFalse(new CommandShooterStopInstant(m_SubsystemShooter));
    primaryController.y().onFalse(new CommandShooterStopInstant(m_SubsystemShooter));
    primaryController.x().onFalse(new CommandShooterStopInstant(m_SubsystemShooter));
    //SmartDashboard.putNumber("xposLambda", DataManager.currentRobotPose.get().toPose2d().getTranslation().getX());
    //m_SubsystemSwerveDrivetrain.createTrajectoryFollowCommand(TrajectoryGenerator.generateTrajectory(DataManager.currentRobotPose.get().toPose2d(), blah, new Pose2d(2.5, 2.5, new Rotation2d(0)), Constants.DriveTrain.DriveConstants.AutoConstants.kTrajectoryConfig)).schedule();
  }

  public class CreateAndScheduleCommand extends InstantCommand {
    @Override
    public void execute() {
      Pose2d currentRobotPose = DataManager.currentRobotPose.get().toPose2d();
      SmartDashboard.putNumber("xposLambda", currentRobotPose.getTranslation().getX());
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        DataManager.currentRobotPose.get().toPose2d(), 
        new ArrayList<>(), 
        new Pose2d(4, 4, new Rotation2d(0)), 
        Constants.DriveTrain.DriveConstants.AutoConstants.kTrajectoryConfig
      );
      /*for (double t = 0; t <= 2; t += 0.1) {
        System.out.print("Time: ");
        System.out.println(t);
        System.out.println(trajectory.sample(t).poseMeters.getX());
        System.out.println(trajectory.sample(t).poseMeters.getY());
      }
      */

      ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
      //interiorWaypoints.add(new Translation2d(1, 0));
      
      Trajectory mockSimpleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(4, 0, new Rotation2d()), 
        interiorWaypoints, 
        new Pose2d(1, 0.1, new Rotation2d()), 
        AutoConstants.kTrajectoryConfig
      );

      m_SubsystemSwerveDrivetrain.createTrajectoryFollowCommand(trajectory).schedule();
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new CommandScoreInSpeaker(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemShooter, m_SubsystemFourBar, SpeakerPosition.center),
      
      new CommandSwerveDriveToSetpoint(m_SubsystemSwerveDrivetrain, () -> DataManager.FieldPoses.getNotePositions(0)),
      new CommandFourBarMoveFourBar(m_SubsystemFourBar, SubsystemFourBar.SetPoints.fourBarFullyDeployedPosition),
      new CommandDriveAndIntake(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemFourBar, Constants.FieldConstants.rightBlueWingNote),
      
      new CommandScoreInSpeaker(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemShooter, m_SubsystemFourBar, SpeakerPosition.sourceside),

      new CommandSwerveDriveToSetpoint(m_SubsystemSwerveDrivetrain, () -> DataManager.FieldPoses.getNotePositions(1)),
      new CommandFourBarMoveFourBar(m_SubsystemFourBar, SubsystemFourBar.SetPoints.fourBarFullyDeployedPosition),
      new CommandDriveAndIntake(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemFourBar, Constants.FieldConstants.middleBlueWingNote),

      new CommandScoreInSpeaker(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemShooter, m_SubsystemFourBar, SpeakerPosition.center),

      new CommandSwerveDriveToSetpoint(m_SubsystemSwerveDrivetrain, () -> DataManager.FieldPoses.getNotePositions(2)),
      new CommandFourBarMoveFourBar(m_SubsystemFourBar, SubsystemFourBar.SetPoints.fourBarFullyDeployedPosition),
      new CommandSwerveDriveToSetpoint(m_SubsystemSwerveDrivetrain, Constants.FieldConstants.leftBlueWingNote),
      new CommandDriveAndIntake(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemFourBar, Constants.FieldConstants.leftBlueWingNote),

      new CommandScoreInSpeaker(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemShooter, m_SubsystemFourBar, SpeakerPosition.ampside)
    );
    // return new SequentialCommandGroup(
    //   new CommandScoreInSpeaker(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemShooter, m_SubsystemFourBar, SpeakerPosition.center),
      
    //   new CommandPickupFieldNote(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemFourBar, 0),
      
    //   new CommandScoreInSpeaker(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemShooter, m_SubsystemFourBar, SpeakerPosition.sourceside),

    //   new CommandPickupFieldNote(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemFourBar, 1),

    //   new CommandScoreInSpeaker(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemShooter, m_SubsystemFourBar, SpeakerPosition.center),

    //   new CommandPickupFieldNote(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemFourBar, 2),
      
    //   new CommandScoreInSpeaker(m_SubsystemSwerveDrivetrain, m_subsystemIntake, m_SubsystemShooter, m_SubsystemFourBar, SpeakerPosition.ampside)
    // );
  }
}
