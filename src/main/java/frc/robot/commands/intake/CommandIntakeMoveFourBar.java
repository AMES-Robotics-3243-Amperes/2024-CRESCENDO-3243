package frc.robot.commands.intake;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemIntake.setPoints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class CommandIntakeMoveFourBar extends Command {
  // :> Worth noting the entire point of this command is to be used by another command to set the FourBar to a setpoint
  protected final SubsystemIntake m_Subsystem;
  protected final setPoints m_SetPoint;

  Timer m_timer = new Timer();

  public CommandIntakeMoveFourBar(SubsystemIntake intake, setPoints setPoint) {
    m_Subsystem = intake;
    m_SetPoint = setPoint;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
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

    // TODO: make the setpoint achievable :3
    // return m_Subsystem.getFourBarAtSetPoint(m_SetPoint);
    return m_timer.hasElapsed(1.5);
  }
}
