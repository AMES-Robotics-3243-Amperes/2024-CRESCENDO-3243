// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemShooter;

public class CommandShooterStop extends Command {
  public SubsystemShooter m_SubsystemShooter;

  /** Creates a new CommandShooterStop. */
  public CommandShooterStop(SubsystemShooter subsystemShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_SubsystemShooter = subsystemShooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SubsystemShooter.setFlywheelSpeed(0);
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
    
    double flyWheelSpeed = m_SubsystemShooter.getFlywheelSpeed();

    if (Math.abs(flyWheelSpeed - Constants.ShooterConstants.stopShootSpeed) < Constants.ShooterConstants.stopSpeedRange) {
     return true;
    }  else {
      return false;
    }
  }
}
