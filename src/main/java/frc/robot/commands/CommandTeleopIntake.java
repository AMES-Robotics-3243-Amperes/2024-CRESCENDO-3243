package frc.robot.commands;
import frc.robot.JoyUtil;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemIntake.setPoints;
import edu.wpi.first.wpilibj2.command.Command;
public class CommandTeleopIntake extends Command {

  private final JoyUtil m_Controller;
  protected final SubsystemIntake m_Subsystem;
  /** Creates a new command. */
  public CommandTeleopIntake(SubsystemIntake intake, JoyUtil controller) {
    m_Controller = controller;
    m_Subsystem = intake;
    addRequirements(intake);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // :> Checks which setpoint the driver wants to input
    if (m_Controller.getPOVLeft()) {
      m_Subsystem.setPositionReference(setPoints.position1);
    }
    if (m_Controller.getPOVDown()) {
      m_Subsystem.setPositionReference(setPoints.position2);
    }
    if (m_Controller.getPOVRight()) {
      m_Subsystem.setPositionReference(setPoints.position3);
    }
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
  