package frc.robot.commands.intake;
import frc.robot.JoyUtil;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemIntake.setPoints;
import edu.wpi.first.wpilibj2.command.Command;
public class CommandIntakeTeleop extends Command {

  private final JoyUtil m_Controller;
  protected final SubsystemIntake m_Subsystem;

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
    // ss Activates and Deactivates the Intake when the A button is pressed or unpressed
    if (m_Controller.getAButton()) {
      m_Subsystem.turnOnIntake();
    }
    else {
      m_Subsystem.turnOffIntake();
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
  