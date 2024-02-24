// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatics;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DataManager;
import frc.robot.commands.drivetrain.CommandSwerveDriveToSetpoint;
import frc.robot.commands.intake.CommandFourBarMoveFourBar;
import frc.robot.commands.intake.CommandIntakeRunForTime;
import frc.robot.commands.intake.CommandOuttakeUntilNotSensed;
import frc.robot.commands.shooter.CommandShooterSpinUpSpeaker;
import frc.robot.commands.shooter.CommandShooterStopInstant;
import frc.robot.subsystems.SubsystemFourBar;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemShooter;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;
import frc.robot.utility.SpeakerPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/**
 * Aligns the robot to the speaker and scores in it, returning control once the flywheel is stopped.
 * 
 * @author H!
 */
// :> This one is the original command we used to try and align and score in speaker
public class CommandScoreInSpeaker extends SequentialCommandGroup {

  public CommandScoreInSpeaker(SubsystemSwerveDrivetrain drivetrain, SubsystemIntake intake, SubsystemShooter shooter, SubsystemFourBar fourBar, SpeakerPosition position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // Translation2d speakerLocation = DataManager.FieldPoses.getSpeakerPosition().getTranslation();
    // Translation2d robotLocation = DataManager.currentRobotPose.get().toPose2d().getTranslation();
    // /** Unit vector repersenting the direction from the speaker's center to the current robot position. Will break if the robot is somehow exactly on the speaker. */
    // Translation2d speakerToRobotDirection = robotLocation.minus(speakerLocation).div(robotLocation.minus(speakerLocation).getNorm());
    // Translation2d launchLocation = speakerLocation.plus(speakerToRobotDirection.times(Constants.RobotConstants.speakerRange));
    // Pose2d launchPose = new Pose2d(launchLocation, new Rotation2d(-speakerToRobotDirection.getX(), -speakerToRobotDirection.getY()));
    // There're negatives to switch the direction, so it's robot to speaker instead of speaker to robot direction.

    addCommands(
      new ParallelCommandGroup(
        new CommandSwerveDriveToSetpoint(drivetrain, () -> position.getPose()),
        new CommandFourBarMoveFourBar(fourBar, SubsystemFourBar.setPoints.fourBarNotDeployedPosition),
        new CommandShooterSpinUpSpeaker(shooter),
        new CommandOuttakeUntilNotSensed(intake)
      ),
      new CommandIntakeRunForTime(intake, 0.5),
      new CommandShooterStopInstant(shooter)
    );
  }
}
