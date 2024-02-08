// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.robot.subsystems.SubsystemClimber;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.Climber.ClimberConstants.*;

import com.revrobotics.RelativeEncoder;

public class CommandClimberAutoClimb extends Command {
  // ££ Subsystem
  SubsystemClimber m_subsystemClimber;

  // ££ Checks if the command is done
  boolean commandDone = false;


  /** Creates a new CommandAutoClimber. */
  public CommandClimberAutoClimb(SubsystemClimber subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystemClimber = subsystem;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    commandDone = m_subsystemClimber.autoRunClimber();
    // ££ Add safety for if the command stops but they keep rising
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (commandDone) {
      return true;
    } else {
      return false;
    }
  }
}
