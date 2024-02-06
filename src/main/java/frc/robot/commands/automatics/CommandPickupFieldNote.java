// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatics;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.DataManager;
import frc.robot.commands.drivetrain.CommandSwerveFollowTrajectory;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CommandPickupFieldNote extends SequentialCommandGroup {
  /** Creates a new CommandPickupFieldNote. */
  public CommandPickupFieldNote(SubsystemSwerveDrivetrain drivetrain, int targetNote) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    Pose2d start = DataManager.currentRobotPose.get().toPose2d();
    Translation2d startLocation = start.getTranslation();
    Pose2d target = DataManager.FieldPoses.getNotePositions(targetNote);
    Translation2d targetLocation = target.getTranslation();

    Translation2d movement = targetLocation.minus(startLocation);
    Rotation2d overNoteDirection = new Rotation2d(movement.getX(), movement.getY());

    double preparatoryDistance = Constants.RobotConstants.frameWidth*Math.sqrt(2)/2. + 0.05; // Should be a little over around half the frame diagonal width

    Pose2d between = new Pose2d(
      targetLocation.minus(movement.div(movement.getNorm()).times(preparatoryDistance)),
      overNoteDirection
    );

    addCommands(
      new CommandSwerveFollowTrajectory(drivetrain, TrajectoryGenerator.generateTrajectory(Arrays.asList(
        start, between, new Pose2d(target.getTranslation(), overNoteDirection)
      ), Constants.DriveTrain.DriveConstants.AutoConstants.kTrajectoryConfig))
    );
  }
}
