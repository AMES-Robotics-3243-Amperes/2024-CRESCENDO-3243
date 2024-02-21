// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatics;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.DataManager;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestGroup extends SequentialCommandGroup {
  /** Creates a new TestGroup. */
  public TestGroup(SubsystemSwerveDrivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        DataManager.currentRobotPose.get().toPose2d(), 
        new ArrayList<>(), 
        new Pose2d(4, 4, new Rotation2d(0)), 
        Constants.DriveTrain.DriveConstants.AutoConstants.kTrajectoryConfig);

    SmartDashboard.putNumber("somethingX", trajectory.getInitialPose().getX());
    SmartDashboard.putNumber("somethingY", trajectory.getInitialPose().getY());
    addCommands(
      drivetrain.createTrajectoryFollowCommand(trajectory)
    );
  }
}
