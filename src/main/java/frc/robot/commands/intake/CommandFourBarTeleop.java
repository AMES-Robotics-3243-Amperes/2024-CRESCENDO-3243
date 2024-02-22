// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.JoyUtil;
import frc.robot.subsystems.SubsystemFourBar.setPoints;
import frc.robot.subsystems.SubsystemFourBar;

public class CommandFourBarTeleop extends Command {

  private final JoyUtil m_Controller;
  protected final SubsystemFourBar m_Subsystem;
  /** Creates a new CommandFourBarTeleop. */
  public CommandFourBarTeleop(SubsystemFourBar fourBar, JoyUtil controller) {
    m_Controller = controller;
    m_Subsystem = fourBar;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // :> Checks which setpoint the driver wants to input
    // :> Note do not press multiple of these at the same time :(
      if (m_Controller.getPOVLeft()) {
        m_Subsystem.setFourBarPositionReference(setPoints.fourBarNotDeployedPosition);
      }
      if (m_Controller.getPOVDown()) {
        m_Subsystem.setFourBarPositionReference(setPoints.fourBarHalfDeployedPosition);
      }
      if (m_Controller.getPOVRight()) {
        m_Subsystem.setFourBarPositionReference(setPoints.fourBarFullyDeployedPosition);
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
