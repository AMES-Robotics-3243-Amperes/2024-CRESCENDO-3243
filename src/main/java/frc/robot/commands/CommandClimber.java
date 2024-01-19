// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SubsystemClimber;
import frc.robot.JoyUtil;

public class CommandClimber extends Command {

  // ££ Subsystem
  SubsystemClimber m_subsystemClimber;

  // ££ Controller
  JoyUtil m_controller;

  /** Creates a new ClimberCommand. */
  public CommandClimber(SubsystemClimber subsystem, JoyUtil controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystemClimber = subsystem;
    m_controller = controller;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean dPadUp = m_controller.getPOVUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
