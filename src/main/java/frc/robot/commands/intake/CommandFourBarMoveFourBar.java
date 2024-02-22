package frc.robot.commands.intake;
import frc.robot.subsystems.SubsystemFourBar;
import frc.robot.subsystems.SubsystemFourBar.setPoints;
import edu.wpi.first.wpilibj2.command.Command;


public class CommandFourBarMoveFourBar extends Command {
  // :> Worth noting the entire point of this command is to be used by another command to set the FourBar to a setpoint
  protected final SubsystemFourBar m_Subsystem;
  protected final setPoints m_SetPoint;

  public CommandFourBarMoveFourBar(SubsystemFourBar intake, setPoints setPoint) {
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
    // ss end the command if the FourBar is at the SetPoint
    return m_Subsystem.getFourBarAtSetPoint(m_SetPoint);
  }
}
