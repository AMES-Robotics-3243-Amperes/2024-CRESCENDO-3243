// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatics;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrain.DriveConstants.AutoConstants;
import frc.robot.DataManager;
import frc.robot.commands.climber.CommandClimberAutoClimb;
import frc.robot.commands.drivetrain.CommandSwerveFollowTrajectory;
import frc.robot.subsystems.SubsystemClimber;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CommandClimb extends SequentialCommandGroup {
  /** Creates a new CommandClimb. */
  public CommandClimb(SubsystemSwerveDrivetrain drivetrain, SubsystemClimber climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CommandSwerveFollowTrajectory(drivetrain, TrajectoryGenerator.generateTrajectory(Arrays.asList(
        DataManager.currentRobotPose.get().toPose2d(), new Pose2d()// TODO fill in stage position
      ), AutoConstants.kTrajectoryConfig)),
      new CommandClimberAutoClimb(climber)
    );
  }
}
