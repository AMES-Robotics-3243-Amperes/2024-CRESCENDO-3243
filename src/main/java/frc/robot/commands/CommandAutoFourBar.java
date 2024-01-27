package frc.robot.commands;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemIntake.setPoints;
import edu.wpi.first.wpilibj2.command.Command;


public class CommandAutoFourBar extends Command {
  // :> Worth noting the entire point of this command is to be used by another command to set the touron to a setpoint
  protected final SubsystemIntake m_Subsystem;
  protected final setPoints m_SetPoint;
  public CommandAutoFourBar(SubsystemIntake intake, setPoints setPoint) {
    m_Subsystem = intake;
    m_SetPoint = setPoint;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // ss Tell the intake to go to the setpoint
    m_Subsystem.setFourBarPositionReference(m_SetPoint);
  }
  @Override
  public void execute() {}
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // ss end the command if the 
    return m_Subsystem.getFourBarAtSetPoint(m_SetPoint);
  }
}
