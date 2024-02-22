// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatics;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.DataManager;
import frc.robot.Constants.DriveTrain.DriveConstants.AutoConstants;
import frc.robot.commands.drivetrain.CommandSwerveDriveToSetpoint;
import frc.robot.commands.intake.CommandIntakeMoveFourBar;
import frc.robot.commands.intake.CommandIntakeRunForTime;
import frc.robot.commands.intake.CommandIntakeUntilNotSensed;
import frc.robot.commands.intake.CommandOuttakeUntilNotSensed;
import frc.robot.commands.plate.CommandPlateMoveToPosition;
import frc.robot.commands.shooter.CommandShooterSpinUpSpeaker;
import frc.robot.commands.shooter.CommandShooterStopInstant;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemPlate;
import frc.robot.subsystems.SubsystemShooter;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/**
 * Aligns the robot to the speaker and scores in it, returning control once the flywheel is stopped.
 * 
 * @author H!
 */
public class CommandScoreInSpeaker extends SequentialCommandGroup {

  public CommandScoreInSpeaker(SubsystemSwerveDrivetrain drivetrain, SubsystemIntake intake, SubsystemShooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Translation2d speakerLocation = DataManager.FieldPoses.getSpeakerPosition().getTranslation();
    Translation2d robotLocation = DataManager.currentRobotPose.get().toPose2d().getTranslation();
    /** Unit vector repersenting the direction from the speaker's center to the current robot position. Will break if the robot is somehow exactly on the speaker. */
    Translation2d speakerToRobotDirection = robotLocation.minus(speakerLocation).div(robotLocation.minus(speakerLocation).getNorm());
    Translation2d launchLocation = speakerLocation.plus(speakerToRobotDirection.times(Constants.RobotConstants.speakerRange));
    Pose2d launchPose = new Pose2d(launchLocation, new Rotation2d(-speakerToRobotDirection.getX(), -speakerToRobotDirection.getY()));
    // There're negatives to switch the direction, so it's robot to speaker instead of speaker to robot direction.

    Pose2d goal = new Pose2d(2.22, 5.1, Rotation2d.fromDegrees(180));

    addCommands(
      new ParallelCommandGroup(
        new CommandSwerveDriveToSetpoint(drivetrain, goal),
        new CommandIntakeMoveFourBar(intake, SubsystemIntake.setPoints.fourBarNotDeployedPosition),
        new CommandShooterSpinUpSpeaker(shooter)
      ),
      new CommandOuttakeUntilNotSensed(intake),
      new CommandIntakeRunForTime(intake, 0.5),
      new CommandShooterStopInstant(shooter)
    );
  }
}
