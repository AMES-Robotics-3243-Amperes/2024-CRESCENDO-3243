// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SubsystemIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// ss Just a command to access the TurnOnIntake Method of SubsystemIntake
public class CommandIntakeOn extends InstantCommand {

  protected final SubsystemIntake m_SubsystemIntake;
  public CommandIntakeOn(SubsystemIntake subsystemIntake) {
    m_SubsystemIntake = subsystemIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_SubsystemIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SubsystemIntake.turnOnIntake();
  }
}
