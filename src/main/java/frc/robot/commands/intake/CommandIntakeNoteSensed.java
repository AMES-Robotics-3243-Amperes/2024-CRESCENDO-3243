// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DataManager;
import frc.robot.subsystems.SubsystemIntake;

public class CommandIntakeNoteSensed extends Command {
  protected final SubsystemIntake m_SubsystemIntake;

  /** Creates a new CommandIntakeRunNoteColorSensor. */
  public CommandIntakeNoteSensed(SubsystemIntake subsystemIntake) {
    m_SubsystemIntake = subsystemIntake;
    addRequirements(subsystemIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SubsystemIntake.turnOnIntake();
  }
  

  // Called every time the scheduler runs while the command is scheduled. 
  @Override
  public void execute() { 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SubsystemIntake.turnOffIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    if(DataManager.currentNoteStorageSensor.get()){
      return true;
    }
    return false;
  }
}
