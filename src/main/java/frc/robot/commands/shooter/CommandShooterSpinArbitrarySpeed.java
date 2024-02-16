// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemShooter;

public class CommandShooterSpinArbitrarySpeed extends Command {

  protected double m_setFlywheelSpeed;
  protected SubsystemShooter m_subsystemShooter;

  /** Creates a new CommandShooterSpinArbitrarySpeed. */
  public CommandShooterSpinArbitrarySpeed(SubsystemShooter subsystem, double setFlywheelSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystemShooter = subsystem;
    m_setFlywheelSpeed = setFlywheelSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystemShooter.setFlywheelSpeed(m_setFlywheelSpeed);
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
    double flyWheelSpeed = m_subsystemShooter.getFlywheelSpeed();
    if (Math.abs(flyWheelSpeed - m_setFlywheelSpeed) < Constants.ShooterConstants.ampSpeedRange) {
      return true;
    } else {
      return false;
    }
  }
}
