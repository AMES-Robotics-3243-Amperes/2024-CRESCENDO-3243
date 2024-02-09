package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveTrain.ModuleConstants;
import frc.robot.Constants.DriveTrain.DriveConstants.TempConstants;

/**
 * Represents a single module of an {@link SubsystemSwerveDrivetrain}
 * 
 * @author :3
 */
public class SubsystemSwerveModule {

  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;

  private final Rotation2d m_wheelOffset;

  /**
   * Constructs a {@link SubsystemSwerveModule}
   * 
   * @param drivingCANId the id of the {@link CANSparkMax} for driving
   * @param turningCANId the id of the {@link CANSparkMax} for turning
   * @param wheelOffset the offset of the encoder's 0 state
   * 
   * @author :3
   */
  public SubsystemSwerveModule(int drivingCANId, int turningCANId, Rotation2d wheelOffset) {
    // :3 initialize spark maxes
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // :3 factory reset spark maxes to get them to a known state
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // :3 setup encoders
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    // :3 setup pid controllers
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // :3 apply position and velocity conversion factors
    m_drivingEncoder.setPositionConversionFactor(ModuleConstants.EncoderFactors.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.EncoderFactors.kDrivingEncoderVelocityFactor);
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.EncoderFactors.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.EncoderFactors.kTurningEncoderVelocityFactor);

    // :3 invert the turning encoder
    m_turningEncoder.setInverted(ModuleConstants.PhysicalProperties.kTurningEncoderInverted);

    // :3 enable pid wrap on 0 to 2 pi, as the wheels rotate freely
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // :3 set p, i, and d terms for driving
    m_drivingPIDController.setP(ModuleConstants.PIDF.kDrivingP);
    m_drivingPIDController.setI(ModuleConstants.PIDF.kDrivingI);
    m_drivingPIDController.setD(ModuleConstants.PIDF.kDrivingD);
    m_drivingPIDController.setFF(ModuleConstants.PIDF.kDrivingFF);
    m_drivingPIDController.setOutputRange(ModuleConstants.PIDF.kDrivingMinOutput,
      ModuleConstants.PIDF.kDrivingMaxOutput);

    // :3 set p, i, and d terms for turning
    m_turningPIDController.setP(ModuleConstants.PIDF.kTurningP);
    m_turningPIDController.setI(ModuleConstants.PIDF.kTurningI);
    m_turningPIDController.setD(ModuleConstants.PIDF.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.PIDF.kTurningFF);
    m_turningPIDController.setOutputRange(ModuleConstants.PIDF.kTurningMinOutput,
      ModuleConstants.PIDF.kTurningMaxOutput);

    // :3 set idle modes and current limits
    m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // :3 save configurations in case of a brown out
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    // :3 reset driving encoder
    resetEncoder();

    // :3 set wheel offset
    m_wheelOffset = wheelOffset;
  }

  /**
   * Sets the desired state of the module
   *
   * @param desiredState desired {@link SwerveModuleState}
   * 
   * @author :3
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // :3 apply wheel angular offset
    SwerveModuleState correctedState = new SwerveModuleState();

    correctedState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedState.angle = desiredState.angle.plus(m_wheelOffset);

    // :3 optimize state to avoid turning more than 90 degrees
    Rotation2d currentAngle = Rotation2d.fromRadians(m_turningEncoder.getPosition());
    SwerveModuleState optimizedState = SwerveModuleState.optimize(correctedState, currentAngle);

    // :3 command driving
    m_drivingPIDController.setReference(optimizedState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
  }

  /**
   * Resets the module's driving encoder
   * 
   * @author :3
   */
  public void resetEncoder() {
    m_drivingEncoder.setPosition(0);
  }

  /**
   * @return the {@link SwerveModulePosition} of the module
   * 
   * @author :3
   */
  public SwerveModulePosition getPosition() {
    double position = m_drivingEncoder.getPosition();
    Rotation2d rotation = new Rotation2d(m_turningEncoder.getPosition() - m_wheelOffset.getRadians());

    return new SwerveModulePosition(position, rotation);
  }

  /**
   * @return Gets the output motor current of an inputted motor
   * 
   * @author :>
   */
  public double getMotorOutputCurrent() {
    double outputCurrent = m_drivingSparkMax.getOutputCurrent();
    return outputCurrent;
  }

  /**
   * @return if either of the motors are too hot
   * 
   * @author :3
   */
  public boolean isTooHot() {
    return m_drivingSparkMax.getMotorTemperature() > TempConstants.max1650TempCelsius ||
      m_turningSparkMax.getMotorTemperature() > TempConstants.max550TempCelsius;
  }
}