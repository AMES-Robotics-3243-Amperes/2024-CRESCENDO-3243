// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DataManager;
import frc.robot.commands.drivetrain.CommandSwerveDriveToSetpoint;
import frc.robot.commands.intake.CommandFourBarMoveFourBar;
import frc.robot.commands.intake.CommandIntakeRunForTime;
import frc.robot.commands.intake.CommandOuttakeUntilNotSensed;
import frc.robot.commands.shooter.CommandShooterSpinUpAmp;
import frc.robot.commands.shooter.CommandShooterStopInstant;
import frc.robot.subsystems.SubsystemFourBar;
import frc.robot.subsystems.SubsystemIntake;
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
public class CommandScoreInAmp extends SequentialCommandGroup {
  /** Creates a new CommandScoreInSpeaker. */
  public CommandScoreInAmp(SubsystemSwerveDrivetrain drivetrain, SubsystemIntake intake, SubsystemShooter shooter, SubsystemFourBar fourBar) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Pose2d target = DataManager.FieldPoses.getAmpPosition().transformBy(new Transform2d( -.66, 0, new Rotation2d()));

    addCommands(
      new ParallelCommandGroup(
        new CommandSwerveDriveToSetpoint(drivetrain, target),
        new CommandFourBarMoveFourBar(fourBar, SubsystemFourBar.SetPoints.fourBarNotDeployedPosition),
        new CommandShooterSpinUpAmp(shooter),
        new CommandOuttakeUntilNotSensed(intake)
      ),
      new CommandIntakeRunForTime(intake, 0.5),
      new CommandShooterStopInstant(shooter)
    );
  }
}
