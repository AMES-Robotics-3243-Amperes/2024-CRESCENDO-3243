package frc.robot.commands.drivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

/**
 * A command that allows the robot to follow a Trajectory. This command
 * is setup in a way that it is not prone to drift from the correct
 * path, but will lag behind the trajectory as it drives. For an alternative
 * that is more concerned with speed and precision but is prone to drift,
 * see {@link CommandSwerveFollowTrajectoryFluid}.
 * 
 * @author :3
 */
public class CommandSwerveFollowTrajectory extends Command {
  Timer m_timer = new Timer();
  Rotation2d m_desiredRotation;
  Trajectory m_trajectory;
  SubsystemSwerveDrivetrain m_subsystem;

  /**
   * Creates a new CommandSwerveFollowTrajectory.
   * 
   * @param subsystem The {@link SubsystemSwerveDrivetrain} to follow the trajectory with
   * @param trajectory The {@link Trajectory} to follow. See {@link TrajectoryGenerator} for making Trajectories.
   * 
   * @author :3
   */
  public CommandSwerveFollowTrajectory(SubsystemSwerveDrivetrain subsystem, Trajectory trajectory) {
    m_subsystem = subsystem;
    m_trajectory = trajectory;
    m_desiredRotation = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters.getRotation();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = m_timer.get();
    var desiredState = m_trajectory.sample(currentTime);
    m_subsystem.driveWithSetpoint(desiredState.poseMeters.getTranslation(), desiredState.poseMeters.getRotation());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.driveWithSpeeds(new Translation2d(), 0.0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final double trajectory_completion_time_buffer = 0.08;
    return m_timer.get() > m_trajectory.getTotalTimeSeconds() + trajectory_completion_time_buffer;
  }
}