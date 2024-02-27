package frc.robot.commands.intake;
import frc.robot.DataManager;
import frc.robot.JoyUtil;
import frc.robot.subsystems.SubsystemIntake;
import edu.wpi.first.wpilibj2.command.Command;
public class CommandIntakeTeleop extends Command {

  private final JoyUtil m_Controller;
  protected final SubsystemIntake m_Subsystem;
  protected boolean isIntaking = false;
  protected boolean startingNotePresence = false;

  /** Creates a new command. */
  public CommandIntakeTeleop(SubsystemIntake intake, JoyUtil controller) {
    m_Controller = controller;
    m_Subsystem = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ss Activates and Deactivates the Intake when the A button is pressed or unpressed
    if (m_Controller.getAButton()) {
      m_Subsystem.intake();
      isIntaking = true;
      startingNotePresence = DataManager.currentNoteStorageSensor.get();
    } else if (m_Controller.getBButton()) {
      m_Subsystem.outtake();
      isIntaking = false;
    } 
    if (isIntaking && startingNotePresence != DataManager.currentNoteStorageSensor.get()) {
      isIntaking = false;
    }
    if (!m_Controller.getAButton() && !m_Controller.getBButton() && !isIntaking) {
      m_Subsystem.stop();
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
  