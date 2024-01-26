// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SubsystemShooter;

public class CommandShooterTeleopSpeaker extends Command {
  public SubsystemShooter m_SubsystemShooter;

  /** Creates a new CommandShootTeleopAmp. */
  public CommandShooterTeleopSpeaker(SubsystemShooter subsystemShooter) {
    // Use addRequirements() here to declare subsystem dependencies.

    // && Sets the subsystem to equal the subsystem
    m_SubsystemShooter = subsystemShooter;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SubsystemShooter.setFlywheelSpeed(ShooterConstants.speakerShootSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SubsystemShooter.stopFlywheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
