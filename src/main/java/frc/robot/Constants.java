// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class JoyUtilConstants {
    // :3 size of controller deadzone
    public static final double kDeadzone = 0.12;

    // :3 max amount controller output can change per second
    public static final double kRateLimitLeft = 20;
    public static final double kRateLimitRight = 20;

    // :3 curve stuff
    public static final int exponent1 = 1;
    public static final int exponent2 = 3;
    public static final double coeff1 = 0.4;
    public static final double coeff2 = 0.6;

    // :3 fast and slow mode
    public static final double leftTriggerSpeedMultiplier = 1.3;
    public static final double rightTriggerSpeedMultiplier = 0.55;

    // :3 ports
    public static int primaryControllerID = 0;
    public static int secondaryControllerID = 1;
  }

  public static final class PowerManager {
    /* :> This is the (conservative) estimate for the robot's resistance
      *  The real resistance has been found empiracally and is somewhere within the realm of .019 ohms
      *  The reason why we use the conservative estimate is because 
      *  the battery as it ages will change the resistance and it's better to be safe than sorry
    */ 
    public static final double robotResistance = .025;

    // :> These values dictate at what point the robot is at unsafe voltage draw
    public static final double softVoltageCap = 8.5;
    public static final double hardVoltageCap = 7.8;
  }

  public static final class Climber {
    public static final class ClimberConstants {
      // ££ Sets the current at which the motors will stop running on the descent
      public static final double MotorCurrentLimit = 50.0;

      public static final class IDs {
        // ££ Climber ids
        public static final int kMotorOne = 9;
        public static final int kMotorTwo = 10;
      }

      public static final class MotorSpeeds {
        // ££ Sets the speeds of the motors on the rise and fall
        public static final double kRiseSpeed = 0.5;
        public static final double kInitialFallSpeed = -0.5;
      }

      public static final class ClimberPIDFF {
        // ££ Sets the P, I, D, and FF values for the climber motors when they're pulling the robot up
        public static final double kP = 0.1;
        public static final double kI = 0.1;
        public static final double kD = 0.1;
        public static final double kFF = 0.1;
      }

      // ££ Sets the positional reference value
      public static final double kPositionOffset = 20;

      // ££ Sets the position conversion factor
      public static final double kGearRatio = 12;
    }
  }

  public static final class DriveTrain {

    // :3 constants for individual modules
    public static final class ModuleConstants {
      // :3 pid connects at 0 and 2 pi because rotation is continuous
      public static final double kTurningEncoderPositionPIDMinInput = 0; // :3 radians
      public static final double kTurningEncoderPositionPIDMaxInput = Math.PI * 2; // :3 radians

      // :3 idle modes
      public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
      public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

      // :3 current limits
      public static final int kDrivingMotorCurrentLimit = 50; // :3 amps
      public static final int kTurningMotorCurrentLimit = 20; // :3 amps

      // :3 pidf values / min and max outputs
      public static final class PIDF {

        public static final double kDrivingP = 0.2;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 0;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 0.225;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;
      }

      // :3 everything having to do with the physical assembly of the modules
      public static final class PhysicalProperties {

        /**
         * :3 direct quote from rev robotics:
         * <p>
         * The MAXSwerve module can be configured with one of three pinion gears:
         * 12T, 13T, or 14T. This changes the drive speed of the module
         * (a pinion gear with more teeth will result in a robot that drives faster).
         */
        public static final int kDrivingMotorPinionTeeth = 13;

        // :3 if constructed correctly, all modules' turning encoders will be reversed
        public static final boolean kTurningEncoderInverted = true;

        // :3 required for various calculations
        public static final double kWheelDiameterMeters = 0.0762;
      }

      // all the encoder factors
      public static final class EncoderFactors {

        // :3 quote from rev robotics:
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction =
          (45.0 * 22) / (PhysicalProperties.kDrivingMotorPinionTeeth * 15);

        public static final double kDrivingEncoderPositionFactor =
          (PhysicalProperties.kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction;
        public static final double kDrivingEncoderVelocityFactor =
          ((PhysicalProperties.kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0;

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI);
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0;
      }
    }

    public static final class DriveConstants {

      // :> This entire next section is utilized by PowerManager to manage the robots speed/acceleration
      // :3 speed damper (flat constant supplied speed is multiplied by)
      public static final double kDrivingSpeedDamper = 4.5; // :3 meters per second
      public static final double kSlowDrivingSpeedDamper = 4;
      // :> Speed Damper for the rotation of the robot
      public static final double kAngularSpeedDamper = 2.5 * Math.PI; // :3 radians per second

      // :3 the max physical speed of the modules
      // :3 THIS IS NOT THE MAX DRIVING SPEED (but it can and will limit it)
      // :3 (for now, I really have no clue what this should
      // be, so I have it set unreasonably high)
      public static final double kMaxObtainableModuleSpeed = 100;

      /* :> These control how fast the robot can accelerate
        * Most times the problem/what you are looking for is messing with the controller/PIDs
        * PowerManager uses these constants to control how much power the robot is drawing
      */
      public static final double kMaxRotationAcceleration = 3 * Math.PI; // (radians)
      public static final double kMaxDrivingAcceleration = 9;

      // :3 if the gyro is reversed
      public static final boolean kGyroReversed = false;

      // :3 spark max ids
      public static final class IDs {

        // :3 driving ids
        public static final int kFrontLeftDrivingCanId = 3;
        public static final int kRearLeftDrivingCanId = 2;
        public static final int kFrontRightDrivingCanId = 7;
        public static final int kRearRightDrivingCanId = 6;

        // :3 turning ids
        public static final int kFrontLeftTurningCanId = 4;
        public static final int kRearLeftTurningCanId = 1;
        public static final int kFrontRightTurningCanId = 8;
        public static final int kRearRightTurningCanId = 5;
      }

      // :3 absolute encoder offsets (should be multiples of pi / 2
      // :3 if the encoders were zeroed properly in rev client)
      public static final class ModuleOffsets {
        public static final Rotation2d kFrontLeftOffset = Rotation2d.fromRadians(Math.PI * 0.5);
        public static final Rotation2d kFrontRightOffset = Rotation2d.fromRadians(Math.PI);
        public static final Rotation2d kBackLeftOffset = Rotation2d.fromRadians(0);
        public static final Rotation2d kBackRightOffset = Rotation2d.fromRadians(Math.PI * 1.5);
      }

      // :3 things involving the physical setup of the chassis
      public static final class ChassisKinematics {

        // :3 distance between centers of right and left wheels on robot
        public static final double kRobotWidth = Units.inchesToMeters(15);
        // :3 distance between front and back wheels on robot
        public static final double kRobotLength = Units.inchesToMeters(15);

        // :3 kinematics (defined with above constants)
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
          new Translation2d(kRobotLength / 2, kRobotWidth / 2), new Translation2d(kRobotLength / 2, -kRobotWidth / 2),
          new Translation2d(-kRobotLength / 2, kRobotWidth / 2),
          new Translation2d(-kRobotLength / 2, -kRobotWidth / 2));
      }

      // :3 max temperatures for the drive train motors
      public static final class TempConstants {
        public static final double max550TempCelsius = 100;
        public static final double max1650TempCelsius = 100;
      }

      // :3 auto-movement configuration
      public static final class AutoConstants {
        // :3 turning stuff
        public static final double kMaxAngularVelocityRadians = 2;
        public static final double kMaxAngularAccelerationRadians = 4;

        public static final double kTuringP = 7;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;

        public static final TrapezoidProfile.Constraints kTurningConfig =
          new TrapezoidProfile.Constraints(kMaxAngularVelocityRadians, kMaxAngularAccelerationRadians);

        public static final ProfiledPIDController kTurningPID =
          new ProfiledPIDController(kTuringP, kTurningI, kTurningD, kTurningConfig);

        // :3 driving setpoint stuff
        public static final double kMaxSetpointVelocity = 0.8;
        public static final double kMaxSetpointAcceleration = 0.6;

        public static final double kSetpointP = 1.5;
        public static final double kSetpointI = 0;
        public static final double kSetpointD = 0;

        public static final double kSetpointMaxIGain = 1;
        public static final double kSetpointMinIGain = -kSetpointMaxIGain;

        public static final PIDController kSetpointPID = new PIDController(kSetpointP, kSetpointI, kSetpointD);

        public static final TrajectoryConfig kTrajectoryConfig =
          new TrajectoryConfig(kMaxSetpointVelocity, kMaxSetpointAcceleration);
      }
    }
  }

  public static final class LED {
    //&& These are the constants for the LEDSubsystem.
    //&& They can be moved to JSON later, I just don't know how to do that yet.

    //&& TODO: Set the correct value for the PWM port, because I don't know if 0 is correct.
    public static final int pwmPort = 0;
  }
  public static final class PhotonVision {
    //TODO replace placeholders (maybe done H!)
    public static final String cameraName = "Backward_Global_Camera";

    // :> TODO Replace this with the 2024 field file when season starts
    public static final String fieldLayoutPath = "./2023-JailbreakJamboree-AprilTagLayout.json";
    // :> TODO Replace this position with the actual position on the new chassis once season starts
    public static final Pose3d cameraPosition =
      new Pose3d(new Translation3d(25.0 / 100, -.22, 14.4 / 100), new Rotation3d());
    public static final Transform3d robotToCamera = new Transform3d(new Pose3d(), cameraPosition);
  }

  public static final class ShooterConstants {
    
    public static final double ampShootSpeed = 0.5;
    public static final double speakerShootSpeed = 0.5;
  }
}
