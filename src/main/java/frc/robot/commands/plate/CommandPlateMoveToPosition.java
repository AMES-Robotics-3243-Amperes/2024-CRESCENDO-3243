// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.plate;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemPlate;
import frc.robot.subsystems.SubsystemPlate.Position;

public class CommandPlateMoveToPosition extends Command {
  SubsystemPlate m_subsystem;
  SubsystemPlate.Position m_position;

  public CommandPlateMoveToPosition(SubsystemPlate subsystem, Position position) {
    this.m_subsystem = subsystem;
    this.m_position = position;

    addRequirements(subsystem);

    subsystem.tab.add(this);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setPosition(m_position);
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
    return m_subsystem.getDiscretePosition() == m_position;
  }
}
