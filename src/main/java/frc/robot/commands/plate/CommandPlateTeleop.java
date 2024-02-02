// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.plate;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.JoyUtil;
import frc.robot.subsystems.SubsystemPlate;
import static frc.robot.Constants.Plate.*;

public class CommandPlateTeleop extends Command {
  SubsystemPlate m_subsystem;
  JoyUtil m_controller;
  /** Creates a new CommandPlateTeleop. */
  public CommandPlateTeleop(SubsystemPlate subsystem, JoyUtil controller) {
    m_subsystem = subsystem;
    m_controller = controller;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setPosition(m_subsystem.getNumericPosition() + manualSpeedFactor * m_controller.getRightY());
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
