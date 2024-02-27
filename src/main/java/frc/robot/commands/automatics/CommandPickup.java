// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatics;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.CommandFourBarMoveFourBar;
import frc.robot.commands.intake.CommandIntakeUntilSensed;
import frc.robot.subsystems.SubsystemFourBar;
import frc.robot.subsystems.SubsystemIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CommandPickup extends SequentialCommandGroup {
  /** Creates a new CommandPickup. */
  public CommandPickup(SubsystemIntake intake, SubsystemFourBar fourBar) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new CommandFourBarMoveFourBar(fourBar, SubsystemFourBar.SetPoints.fourBarFullyDeployedPosition),
        new CommandIntakeUntilSensed(intake)
      ),
      new CommandFourBarMoveFourBar(fourBar, SubsystemFourBar.SetPoints.fourBarNotDeployedPosition)
    );
  }
}
