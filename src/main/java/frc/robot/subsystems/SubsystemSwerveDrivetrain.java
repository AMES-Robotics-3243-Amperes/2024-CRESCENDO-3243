package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.DataManager;
import frc.robot.IMUWrapper;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.Constants.DriveTrain.DriveConstants.ChassisKinematics;
import frc.robot.utility.PowerManager;

/**
 * Subsystem for a swerve drivetrain
 * 
 * @author :3
 */
public class SubsystemSwerveDrivetrain extends SubsystemBase {

  // :3 create swerve modules
  private final SubsystemSwerveModule m_frontLeft = new SubsystemSwerveModule(DriveConstants.IDs.kFrontLeftDrivingCanId,
    DriveConstants.IDs.kFrontLeftTurningCanId, DriveConstants.ModuleOffsets.kFrontLeftOffset);

  private final SubsystemSwerveModule m_frontRight = new SubsystemSwerveModule(DriveConstants.IDs.kFrontRightDrivingCanId,
    DriveConstants.IDs.kFrontRightTurningCanId, DriveConstants.ModuleOffsets.kFrontRightOffset);

  private final SubsystemSwerveModule m_rearLeft = new SubsystemSwerveModule(DriveConstants.IDs.kRearLeftDrivingCanId,
    DriveConstants.IDs.kRearLeftTurningCanId, DriveConstants.ModuleOffsets.kBackLeftOffset);

  private final SubsystemSwerveModule m_rearRight = new SubsystemSwerveModule(DriveConstants.IDs.kRearRightDrivingCanId,
    DriveConstants.IDs.kRearRightTurningCanId, DriveConstants.ModuleOffsets.kBackRightOffset);

  /** :3 odometry for tracking robot pose */
  SwerveDriveOdometry m_odometry;

  // :3 imu for odometry
  IMUWrapper m_imu = new IMUWrapper();

  /**
   * Creates a new DriveSubsystem.
   * 
   * @author :3
   */
  public SubsystemSwerveDrivetrain() {
    // :3 initialize odometry
    m_odometry = new SwerveDriveOdometry(ChassisKinematics.kDriveKinematics,
      DataManager.currentRobotPose.get().getRotation().toRotation2d(), getModulePositions());

    // :3 reset module driving encoders
    resetModuleDrivingEncoders();
  }

  /**
   * @param trajectory the {@link Trajectory} to follow. see {@link TrajectoryGenerator} for creating trajectories.
   * 
   * @return A command that follows the provided trajectory
   * 
   * @author :3
   */
  public SwerveControllerCommand createTrajectoryFollowCommand(Trajectory trajectory) {
    return new SwerveControllerCommand(trajectory,
      () -> { return DataManager.currentRobotPose.get().toPose2d(); },
      ChassisKinematics.kDriveKinematics,
      new PIDController(0, 0, 0),
      new PIDController(0, 0, 0),
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0)),
      () -> { return trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters.getRotation(); },
      this::setModuleStates,
      this);
  }

  @Override
  public void periodic() {
    m_odometry.update(m_imu.getYaw(), getModulePositions());
    DataManager.currentRobotPose.updateWithOdometry(m_odometry.getPoseMeters());

    PowerManager.frontLeftDrivetrainMotorPresentCurrent = m_frontLeft.getMotorOutputCurrent();
    PowerManager.frontRightDrivetrainMotorPresentCurrent = m_frontRight.getMotorOutputCurrent();
    PowerManager.rearLeftDrivetrainMotorPresentCurrent = m_rearLeft.getMotorOutputCurrent();
    PowerManager.rearRightDrivetrainMotorPresentCurrent = m_rearRight.getMotorOutputCurrent();
  }

  /**
   * Set the swerve modules' desired states
   *
   * @param desiredStates the desired {@link SwerveModuleState}s (front left, front right, rear left, rear right)
   * 
   * @author :3
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // :3 desaturate wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxObtainableModuleSpeed);

    // :3 set the desired states
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Resets the {@link SubsystemSwerveModule}s' driving encoders
   * 
   * @author :3
   */
  public void resetModuleDrivingEncoders() {
    m_frontLeft.resetEncoder();
    m_rearLeft.resetEncoder();
    m_frontRight.resetEncoder();
    m_rearRight.resetEncoder();
  }

  /**
   * @return the positions of the swerve modules
   * 
   * @author :3
   */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[]{
      m_frontLeft.getPosition(), m_frontRight.getPosition(),
      m_rearLeft.getPosition(), m_rearRight.getPosition()
    };
  }

  /**
   * @return if the motors are at safe temperatures
   * 
   * @author :3
   */
  public boolean getMotorsOkTemperature() {
    return !(m_frontLeft.isTooHot() || m_frontRight.isTooHot() || m_rearLeft.isTooHot() || m_rearRight.isTooHot());
  }

  /**
   * @return the heading reported by the gyro
   * 
   * @author :3
   */
  public Rotation2d getGyroHeading() {
    return m_imu.getYaw();
  }
}