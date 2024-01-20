package frc.robot.commands;
import frc.robot.JoyUtil;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemIntake.setPoints;
import frc.robot.commands.CommandHoldIntake;
import edu.wpi.first.wpilibj2.command.Command;
public class CommandTeleopIntake extends Command {

  private final JoyUtil m_controller;
  protected final SubsystemIntake m_subsystem;
  protected final CommandHoldIntake m_CommandHoldIntake;
  /** Creates a new command. */
  public CommandTeleopIntake(SubsystemIntake intake, JoyUtil controller, CommandHoldIntake commandHoldIntake) {
    m_controller = controller;
    m_subsystem = intake;
    m_CommandHoldIntake = commandHoldIntake;
    addRequirements(intake);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        m_controller.a().whileTrue(m_CommandHoldIntake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // :> Checks which setpoint the driver wants to input
    if (m_controller.getPOVLeft()) {
      m_subsystem.setPositionReference(setPoints.position1);
    }
    if (m_controller.getPOVDown()) {
      m_subsystem.setPositionReference(setPoints.position2);
    }
    if (m_controller.getPOVRight()) {
      m_subsystem.setPositionReference(setPoints.position3);
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
  