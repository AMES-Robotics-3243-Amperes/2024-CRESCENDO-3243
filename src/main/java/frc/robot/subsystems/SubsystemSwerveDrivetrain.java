package frc.robot.subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DataManager;
import frc.robot.IMUWrapper;
import frc.robot.utility.PowerManager;
import frc.robot.utility.SubsystemBaseTestable;
import frc.robot.utility.Test;
import frc.robot.utility.TestUtil;
import frc.robot.Constants.DriveTrain.DriveConstants;
import frc.robot.Constants.DriveTrain.DriveConstants.AutoConstants;
import frc.robot.utility.TranslationRateLimiter;

/**
 * Subsystem for a swerve drivetrain
 * 
 * @author :3
 */
public class SubsystemSwerveDrivetrain extends SubsystemBaseTestable {

  // :3 create swerve modules
  private final SubsystemSwerveModule m_frontLeft = new SubsystemSwerveModule(DriveConstants.IDs.kFrontLeftDrivingCanId,
    DriveConstants.IDs.kFrontLeftTurningCanId, DriveConstants.ModuleOffsets.kFrontLeftOffset);

  private final SubsystemSwerveModule m_frontRight = new SubsystemSwerveModule(DriveConstants.IDs.kFrontRightDrivingCanId,
    DriveConstants.IDs.kFrontRightTurningCanId, DriveConstants.ModuleOffsets.kFrontRightOffset);

  private final SubsystemSwerveModule m_rearLeft = new SubsystemSwerveModule(DriveConstants.IDs.kRearLeftDrivingCanId,
    DriveConstants.IDs.kRearLeftTurningCanId, DriveConstants.ModuleOffsets.kBackLeftOffset);

  private final SubsystemSwerveModule m_rearRight = new SubsystemSwerveModule(DriveConstants.IDs.kRearRightDrivingCanId,
    DriveConstants.IDs.kRearRightTurningCanId, DriveConstants.ModuleOffsets.kBackRightOffset);
  // :3 driving speeds/information
  private Translation2d m_speeds;
  private Boolean m_drivingFieldRelative;
  private Double m_turnSpeedRadians;

  private Translation2d m_fieldSetpoint;
  private Rotation2d m_rotationSetpoint;

  // :3 rotation pid and rate limit
  private ProfiledPIDController m_rotationPidControllerRadians = AutoConstants.kTurningPID;
  private SlewRateLimiter m_roatationLimiter =
    new SlewRateLimiter(DriveConstants.kMaxRotationAcceleration, -DriveConstants.kMaxRotationAcceleration, 0);

  // :3 setpoint pid and driving rate limiter
  private PIDController m_setpointXPidController = AutoConstants.kSetpointPID;
  private PIDController m_setpointYPidController = AutoConstants.kSetpointPID;
  private TranslationRateLimiter m_setpointPidGoalRateLimiter =
    new TranslationRateLimiter(getPosition(), AutoConstants.kMaxSetpointVelocity);
  private TranslationRateLimiter m_drivingRateLimiter =
    new TranslationRateLimiter(new Translation2d(), DataManager.currentAccelerationConstant.get());

  // :3 imu
  private final IMUWrapper m_imuWrapper = new IMUWrapper();

  /** :3 odometry for tracking robot pose */
  SwerveDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   * 
   * @author :3
   */
  public SubsystemSwerveDrivetrain() {
    // :3 initialize odometry
    m_odometry = new SwerveDriveOdometry(DriveConstants.ChassisKinematics.kDriveKinematics,
      DataManager.currentRobotPose.get().getRotation().toRotation2d(), getModulePositions());

    // :3 configure turning pid
    m_rotationPidControllerRadians.enableContinuousInput(-Math.PI, Math.PI);
    resetRotationPID();

    // :3 configure setpoint pids
    m_setpointXPidController.setIntegratorRange(AutoConstants.kSetpointMinIGain, AutoConstants.kSetpointMaxIGain);
    m_setpointYPidController.setIntegratorRange(AutoConstants.kSetpointMinIGain, AutoConstants.kSetpointMaxIGain);

    // :3 reset module driving encoders
    resetModuleDrivingEncoders();
  }

  @Override
  public void doPeriodic() {
    m_drivingRateLimiter.changeLimit(DataManager.currentAccelerationConstant.get());

    // :3 update odometry and feed that information into DataManager
    m_odometry.update(m_imuWrapper.getYaw(), getModulePositions());
    DataManager.currentRobotPose.updateWithOdometry(m_odometry.getPoseMeters());

    // :3 these are the raw speeds of the robot,
    // and will be assigned in the next 2 if statements
    double rawXSpeed = 0.0;
    double rawYSpeed = 0.0;
    double rawRotationSpeed = 0.0;
    boolean fieldRelative = false;

    // :3 initialize movement speeds
    if (m_fieldSetpoint != null) {
      Translation2d fixedVelocitySetpoint =
        m_setpointPidGoalRateLimiter.calculate(m_fieldSetpoint.minus(getPosition()));
      double xGoal = fixedVelocitySetpoint.getX();
      double yGoal = fixedVelocitySetpoint.getY();

      rawXSpeed = m_setpointXPidController.calculate(getPosition().getX(), xGoal);
      rawYSpeed = m_setpointYPidController.calculate(getPosition().getY(), yGoal);

      double speed = Math.sqrt(rawXSpeed * rawXSpeed + rawYSpeed * rawYSpeed);
      if (speed > AutoConstants.kMaxSetpointVelocity) {
        rawXSpeed *= (AutoConstants.kMaxSetpointVelocity / speed);
        rawYSpeed *= (AutoConstants.kMaxSetpointVelocity / speed);
      }

      fieldRelative = true;
    } else if (m_speeds != null && m_drivingFieldRelative != null) {
      rawXSpeed = m_speeds.getX();
      rawYSpeed = m_speeds.getY();
      fieldRelative = m_drivingFieldRelative;
    }

    // :3 initialize rotation speeds
    if (m_rotationSetpoint != null) {
      rawRotationSpeed =
        m_rotationPidControllerRadians.calculate(getHeading().getRadians(), m_rotationSetpoint.getRadians());
    } else if (m_turnSpeedRadians != null) {
      rawRotationSpeed = m_turnSpeedRadians;
    }

    // :3 convert to field relative speeds for rate limiting
    double rawFieldRelativeXSpeed;
    double rawFieldRelativeYSpeed;
    if (!fieldRelative) {
      rawFieldRelativeXSpeed = rawXSpeed * getHeading().getCos() - rawYSpeed * getHeading().getSin();
      rawFieldRelativeYSpeed = rawXSpeed * getHeading().getSin() + rawYSpeed * getHeading().getCos();
    } else {
      rawFieldRelativeXSpeed = rawXSpeed;
      rawFieldRelativeYSpeed = rawYSpeed;
    }
    
    // :3 rate limit
    rawRotationSpeed = m_roatationLimiter.calculate(rawRotationSpeed);
    Translation2d rawFieldRelativeSpeeds = 
      m_drivingRateLimiter.calculate(new Translation2d(rawFieldRelativeXSpeed, rawFieldRelativeYSpeed));
    rawFieldRelativeXSpeed = rawFieldRelativeSpeeds.getX();
    rawFieldRelativeYSpeed = rawFieldRelativeSpeeds.getY();

    // :3 reset pids
    if (m_rotationSetpoint == null) {
      resetRotationPID();
    }
    if (m_fieldSetpoint == null) {
      m_setpointXPidController.reset();
      m_setpointYPidController.reset();
      m_setpointPidGoalRateLimiter.reset(getPosition());
    }

    // :3 get module states
    SwerveModuleState[] swerveModuleStates = DriveConstants.ChassisKinematics.kDriveKinematics.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(rawFieldRelativeXSpeed, rawFieldRelativeYSpeed, rawRotationSpeed, getHeading()));

    setModuleStates(swerveModuleStates);

    // :> Sets the power manager variables equal to the current current outputs
    PowerManager.frontLeftDrivetrainMotorPresentCurrent = getDriveFrontLeftCurrent();
    PowerManager.frontRightDrivetrainMotorPresentCurrent = getDriveFrontRightCurrent();
    PowerManager.rearLeftDrivetrainMotorPresentCurrent = getDriveRearLeftCurrent();
    PowerManager.rearRightDrivetrainMotorPresentCurrent = getDriveRearRightCurrent();

  }

  /**
   * Sets the speeds the robot should drive at.
   * Nulls are okay and will be used as 0's.
   *
   * @param speeds a vector representing the speeds of the robot
   * @param rotationSpeed angular rate of the robot in radians
   * @param drivingFieldRelative whether the provided x and y speeds are relative to the field
   * 
   * @author :3
   */
  public void driveWithSpeeds(Translation2d speeds, Double rotationSpeed, Boolean drivingFieldRelative) {
    m_speeds = speeds;
    m_turnSpeedRadians = rotationSpeed;
    m_drivingFieldRelative = drivingFieldRelative;

    // :3 if something's being controlled with speeds, null the setpoints
    if (speeds != null) {
      m_fieldSetpoint = null;
    }

    if (rotationSpeed != null) {
      m_rotationSetpoint = null;
    }
  }

  /**
   * drive the robot with setpoints
   *
   * @param xSpeed speed of the robot in the x direction (forward)
   * @param ySpeed speed of the robot in the y direction (sideways)
   * @param rotation or goal of field relative driving
   * @param fieldRelative whether the provided x and y speeds are relative to the field
   * 
   * @author :3
   */
  public void driveWithSetpoint(Translation2d fieldSetpoint, Rotation2d rotationSetpoint) {
    m_fieldSetpoint = fieldSetpoint;
    m_rotationSetpoint = rotationSetpoint;

    // :3 if something's being controlled with setpoints, null the speeds
    if (fieldSetpoint != null) {
      m_speeds = null;
    }

    if (rotationSetpoint != null) {
      m_turnSpeedRadians = null;
    }
  }

  /**
   * Set the swerve modules' desired states
   *
   * @param desiredStates the desired {@link SwerveModuleState}s
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
   * Private helper function
   * 
   * @return the robot's heading
   * 
   * @author :3
   */
  private Rotation2d getHeading() {
    double rawAngleRadians = DataManager.currentRobotPose.get().getRotation().toRotation2d().getRadians();
    return Rotation2d.fromRadians(MathUtil.angleModulus(rawAngleRadians));
  }

  /**
   * Private helper function
   * 
   * @return the robot's position
   * 
   * @author :3
   */
  private Translation2d getPosition() {
    return DataManager.currentRobotPose.get().getTranslation().toTranslation2d();
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
   * resets the rotation pid
   * 
   * @author :3
   */
  private void resetRotationPID() {
    m_rotationPidControllerRadians.reset(getHeading().getRadians());
  }

  /*  :> The reason why I'm doing all of this is because only SubsystemSwerve has access to all the motors directly
   *  I'll be using these in periodic to set motor currents in power manager to make fail safes and redirect current.
  */
  /**
   * @return frontLeft Motors current output at a given moment
   *
   * @author :>
  */
  public double getDriveFrontLeftCurrent() {
    return (m_frontLeft.getMotorOutputCurrent());
  }

  /**
   * @return frontRight Motors current output at a given moment
   *
   * @author :>
  */
  public double getDriveFrontRightCurrent() {
    return (m_frontRight.getMotorOutputCurrent());
  }

  /**
   * @return rearLeft Motors current output at a given moment
   *
   * @author :>
  */
  public double getDriveRearLeftCurrent() {
    return (m_rearLeft.getMotorOutputCurrent());
  }

  /**
   * @return rearRight Motors current output at a given moment
   *
   * @author :>
  */
  public double getDriveRearRightCurrent() {
    return (m_rearRight.getMotorOutputCurrent());
  }

  @Override
  public Test[] getTests() {
    return new Test[]{new ExampleDependentTest(), instanceExampleFailingTest};
  }

  protected class ExampleTest implements Test {
  @Override public void testPeriodic() {/*TestUtil.assertEquals(2,3);*/}
    @Override public boolean testIsDone() {return true;}
    @Override public void setupPeriodic() {}
    @Override public boolean setupIsDone() {return true;}
    @Override public void closedownPeriodic() {}
    @Override public boolean closedownIsDone() {return true;}

    @Override public String getName() {return "Example Test";}

    @Override public Test[] getDependencies() {return new Test[0];}
  }

  protected class ExampleFailingTest implements Test {

    @Override public void testPeriodic() {TestUtil.assertEquals(2+2,7);}
    @Override public boolean testIsDone() {return true;}
    @Override public void setupPeriodic() {}
    @Override public boolean setupIsDone() {return true;}
    @Override public void closedownPeriodic() {}
    @Override public boolean closedownIsDone() {return true;}

    @Override public String getName() {return "Example Failing Test";}

    @Override public Test[] getDependencies() {return new Test[0];}
  }
  public ExampleFailingTest instanceExampleFailingTest = new ExampleFailingTest();

  protected class ExampleDependentTest implements Test {
    @Override public void testPeriodic() {}
    @Override public boolean testIsDone() {return true;}
    @Override public void setupPeriodic() {}
    @Override public boolean setupIsDone() {return true;}
    @Override public void closedownPeriodic() {}
    @Override public boolean closedownIsDone() {return true;}

    @Override public String getName() {return "Example Dependent Test";}

    protected Test[] dependencies = new Test[]{new ExampleTest(), instanceExampleFailingTest};

    @Override public Test[] getDependencies() {return dependencies;}
    @Override public boolean[] getDependencySuccessRequirements() {
      return new boolean[]{true, true};
    }
  }
}