// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

/**
 * A simple command that drives a {@link SubsystemSwerveDrivetrain} to a {@link Pose2d}.
 * 
 * @author :3
 */
public class CommandSwerveDriveToPointCommand extends Command {
  private SubsystemSwerveDrivetrain m_subsystem;
  private Pose2d m_setpoint;

  /** 
   * @param subsystem The {@link SubsystemSwerveDrivetrain} to control
   * @param setpoint The {@link Pose2d} to drive it to
   * 
   * @author :3
   */
  public CommandSwerveDriveToPointCommand(SubsystemSwerveDrivetrain subsystem, Pose2d setpoint) {
    m_subsystem = subsystem;
    m_setpoint = setpoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.driveWithSetpoint(m_setpoint.getTranslation(), m_setpoint.getRotation());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
