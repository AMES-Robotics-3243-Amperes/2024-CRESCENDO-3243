// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automatics;

import java.util.Arrays;

import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DataManager;
import frc.robot.Constants.DriveTrain.DriveConstants.AutoConstants;
import frc.robot.commands.drivetrain.CommandSwerveFollowTrajectory;
import frc.robot.commands.intake.CommandIntakeMoveFourBar;
import frc.robot.commands.intake.CommandIntakeNoteNotSensed;
import frc.robot.commands.intake.CommandIntakeRunForTime;
import frc.robot.commands.plate.CommandPlateMoveToPosition;
import frc.robot.commands.shooter.CommandShooterSpinUpAmp;
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
public class CommandScoreInAmp extends SequentialCommandGroup {
  /** Creates a new CommandScoreInSpeaker. */
  public CommandScoreInAmp(SubsystemSwerveDrivetrain drivetrain, SubsystemIntake intake, SubsystemShooter shooter, SubsystemPlate plate) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new CommandSwerveFollowTrajectory(drivetrain, TrajectoryGenerator.generateTrajectory(Arrays.asList(
          DataManager.currentRobotPose.get().toPose2d(), DataManager.FieldPoses.getAmpPosition()
        ), AutoConstants.kTrajectoryConfig)),
        new CommandIntakeMoveFourBar(intake, SubsystemIntake.setPoints.fourBarNotDeployedPosition),
        new CommandShooterSpinUpAmp(shooter),
        new CommandPlateMoveToPosition(plate, SubsystemPlate.Position.kAmp)
      ),
      new CommandIntakeNoteNotSensed(intake),
      new CommandShooterStopInstant(shooter)
    );
  }
}
