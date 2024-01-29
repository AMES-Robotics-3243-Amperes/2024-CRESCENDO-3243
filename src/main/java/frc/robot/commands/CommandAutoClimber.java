// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SubsystemClimber;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandAutoClimber extends Command {
  // ££ Subsystem
  SubsystemClimber m_subsystemClimber;

  // ££ Limit Switche values
  DigitalInput limitSwitch;
  boolean limitSwitchTripped = false;


  /** Creates a new CommandAutoClimber. */
  public CommandAutoClimber(SubsystemClimber subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystemClimber = subsystem;
    
    limitSwitch = new DigitalInput(0);

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limitSwitch.get()) {
      limitSwitchTripped = true;
    }

    if (!limitSwitchTripped) {
      m_subsystemClimber.runClimber(true, false);
    } else {
      m_subsystemClimber.runClimber(false, true);
    }
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
