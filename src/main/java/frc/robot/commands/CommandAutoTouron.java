package frc.robot.commands;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemIntake.setPoints;
import edu.wpi.first.wpilibj2.command.Command;


public class CommandAutoTouron extends Command {
  // :> Worth noting the entire point of this command is to be used by another command to set the touron to a setpoint
  protected final SubsystemIntake m_subsystem;
  protected final setPoints m_setPoint;
  public CommandAutoTouron(SubsystemIntake intake, setPoints setPoint) {
    m_subsystem = intake;
    m_setPoint = setPoint;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setPositionReference(m_setPoint);
  }
  @Override
  public void execute() {}
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.getTouronAtPosition(m_setPoint);
  }
}
