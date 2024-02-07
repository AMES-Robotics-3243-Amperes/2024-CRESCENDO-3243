package frc.robot.commands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

/**
 * A command that allows the robot to follow a Trajectory. This command
 * is setup in a way that prioritizes speed and precision, but is prone to
 * drift from the correct path. For an alternative that does not experience
 * drift but may lag behind the trajectory, see {@link CommandSwerveFollowTrajectory}.
 * 
 * @author :3
 */
public class CommandSwerveFollowTrajectoryFluid extends Command {
  Timer m_timer = new Timer();
  Rotation2d m_desiredRotation;
  Trajectory m_trajectory;
  SubsystemSwerveDrivetrain m_subsystem;

  /**
   * Creates a new CommandSwerveFollowTrajectoryFluid.
   * 
   * @param subsystem The {@link SubsystemSwerveDrivetrain} to follow the trajectory with
   * @param trajectory The {@link Trajectory} to follow. See {@link TrajectoryGenerator} for making Trajectories.
   * 
   * @author :3
   */
  public CommandSwerveFollowTrajectoryFluid(SubsystemSwerveDrivetrain subsystem, Trajectory trajectory) {
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
    var currentState = m_trajectory.sample(currentTime);

    final double two_loop_time = 0.002;
    var nextState = m_trajectory.sample(currentTime + two_loop_time);

    Translation2d unitVelocity = nextState.poseMeters.getTranslation().minus(currentState.poseMeters.getTranslation());
    double directionNorm = unitVelocity.getNorm();
    unitVelocity = unitVelocity.div(directionNorm).rotateBy(Rotation2d.fromDegrees(90));

    // :3 if the position doesn't change from one state to the next, a division by zero happens
    // and the robot hurls itself to the nearest wall with no regard to the saftey of itself,
    // the building it's in, or the people in it. yes, I realized this the hard way
    final double normIsZeroRange = 0.002;
    if (Math.abs(directionNorm) < normIsZeroRange) {
      unitVelocity = new Translation2d();
    }

    Rotation2d rotationChange = nextState.poseMeters.getRotation().minus(currentState.poseMeters.getRotation());
    double rotationSpeed = rotationChange.getRadians() / two_loop_time;

    m_subsystem.driveWithSpeeds(unitVelocity.times(currentState.velocityMetersPerSecond), rotationSpeed, true);
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