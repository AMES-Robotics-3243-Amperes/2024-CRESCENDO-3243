// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drivetrain.CommandSwerveDriveToSetpoint;
import frc.robot.commands.intake.CommandFourBarMoveFourBar;
import frc.robot.commands.intake.CommandIntakeRunForTime;
import frc.robot.commands.intake.CommandIntakeUntilNotSensed;
import frc.robot.commands.intake.CommandIntakeUntilSensed;
import frc.robot.subsystems.SubsystemFourBar;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemShooter;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CommandDriveAndIntake extends ParallelCommandGroup {
  /** Creates a new CommandDriveAndIntake. */
  public CommandDriveAndIntake(SubsystemSwerveDrivetrain drivetrain, SubsystemIntake intake, SubsystemFourBar fourBar, Pose2d goal) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CommandFourBarMoveFourBar(fourBar, SubsystemFourBar.SetPoints.fourBarFullyDeployedPosition),
      new ParallelCommandGroup(
        new CommandIntakeRunForTime(intake, 1.1),
        new CommandSwerveDriveToSetpoint(drivetrain, goal)
      )
    );
  }
}
