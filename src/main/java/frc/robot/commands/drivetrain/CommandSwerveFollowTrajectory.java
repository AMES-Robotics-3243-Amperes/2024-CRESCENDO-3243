package frc.robot.commands.drivetrain;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DataManager;
import frc.robot.Constants.DriveTrain.DriveConstants.AutoConstants;
import frc.robot.subsystems.SubsystemSwerveDrivetrain;

public class CommandSwerveFollowTrajectory extends Command {
  Timer m_timer = new Timer();
  Rotation2d m_desiredRotation;
  Trajectory m_trajectory;
  SubsystemSwerveDrivetrain m_subsystem;
  HolonomicDriveController m_driveController;

  /**
   * Creates a new CommandSwerveFollowTrajectory.
   * 
   * @author :3
   */
  public CommandSwerveFollowTrajectory(SubsystemSwerveDrivetrain subsystem, Trajectory trajectory,
      PIDController xPID, PIDController yPID) {
    m_subsystem = subsystem;
    m_trajectory = trajectory;
    m_desiredRotation = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters.getRotation();
    m_driveController = new HolonomicDriveController(xPID, yPID,
      new ProfiledPIDController(0, 0, 0,
        new TrapezoidProfile.Constraints(AutoConstants.kMaxSetpointVelocity, AutoConstants.kMaxSetpointAcceleration)));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  public CommandSwerveFollowTrajectory(SubsystemSwerveDrivetrain subsystem, Trajectory trajectory) {
    // Unimplemented
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

    ChassisSpeeds targetChassisSpeeds =
      m_driveController.calculate(DataManager.currentRobotPose.get().toPose2d(), desiredState, m_desiredRotation);
    
    Translation2d velocity = new Translation2d(
      targetChassisSpeeds.vxMetersPerSecond, targetChassisSpeeds.vyMetersPerSecond);
    m_subsystem.driveWithSpeeds(velocity, null, false);
    m_subsystem.driveWithSetpoint(null, m_desiredRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > m_trajectory.getTotalTimeSeconds();
  }
}