package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DriveTrain.DriveConstants;

/**
 * A wrapper for an {@link AHRS}
 * 
 * @author :3
 */
public class IMUWrapper {
  private final AHRS m_imu = new AHRS();
  private final boolean m_gyroReversed;

  /**
   * Creates a new IMUSubsystem
   */
  public IMUWrapper() {
    m_gyroReversed = DriveConstants.kGyroReversed;
  }

  /**
   * @return the yaw value that the gyro reads
   * 
   * @author :3
   */
  public Rotation2d getYaw() {
    Rotation2d raw_angle = m_imu.getRotation2d();
    return m_gyroReversed ? raw_angle.times(-1) : raw_angle;
  }

  /**
   * @return the turn rate of the robot
   * 
   * @author :3
   */
  public Rotation2d getTurnRate() {
    Rotation2d rawTurnRateDegrees = Rotation2d.fromDegrees(m_imu.getRate());
    return DriveConstants.kGyroReversed ? rawTurnRateDegrees.times(-1) : rawTurnRateDegrees;
  }
}