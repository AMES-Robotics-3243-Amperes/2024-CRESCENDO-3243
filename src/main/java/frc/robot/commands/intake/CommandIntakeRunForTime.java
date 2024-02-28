// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemIntake;

public class CommandIntakeRunForTime extends Command {

  protected final SubsystemIntake m_SubsystemIntake;
  protected final Timer m_Timer = new Timer();
  protected final double m_Time;

  /**
   * Runs the "subsystemIntake" for "time" seconds
   * @param subsystemIntake
   * @param time The amount of time, in SECONDS, to run the intake for
   * @author ss
   */
  public CommandIntakeRunForTime(SubsystemIntake subsystemIntake, double time) {
    m_SubsystemIntake = subsystemIntake;
    m_Time = time;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystemIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
    m_SubsystemIntake.intake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SubsystemIntake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Timer.hasElapsed(m_Time);
  }
}
