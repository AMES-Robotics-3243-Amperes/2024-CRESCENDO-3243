// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
  /**
   * Constants pertaining to physical parameters of the robot.
   */
  public static final class RobotConstants {
    public static final double speakerRange = Units.inchesToMeters(72);// Inexact number, measurement is needed
    public static final double frameWidth = Units.inchesToMeters(26);
  }

  public static final class FieldConstants {
    /* 
     * :> Worth noting I'm defining forwards to be facing towards Red Alliance (For example: Facing ID 4 )
     * 0,0 is also the bottom left corner of the field looking top down with blue alliance on your left
     * All positions are based off this graph: https://www.desmos.com/calculator/77yiven4dn
     * Positive X is forwards towards red alliance. Positive Y is towards left when you are facing the red alliance
     * Merrick was silly and so I will be doing everything in terms of artificially defined constants
     * Note: we are in meters
    */ 

    public static double fieldHeight = 8.02;
    public static double fieldWidth = 16.54;
    // :> I don't know why he did it this way but heres an artificial constant that I'm extracting off of
    public static double merrickFieldConstant = (2.5399986284);

    // :> All Rotations I'm going to set to be 0 as we don't actually care what direction we approach the notes from, we only need the x and y
    public static Pose2d leftBlueWingNote = new Pose2d(((fieldWidth)- (1.14 * merrickFieldConstant)), ((fieldHeight / 2) - (2 * (.57 * merrickFieldConstant))), new Rotation2d(9));
    public static Pose2d middleBlueWingNote = new Pose2d(((fieldWidth)- (1.14 * merrickFieldConstant)), ((fieldHeight / 2) - (.57 * merrickFieldConstant)), new Rotation2d(0));
    public static Pose2d rightBlueWingNote = new Pose2d(((fieldWidth)- (1.14 * merrickFieldConstant)), (fieldHeight / 2), new Rotation2d(0));
    
    public static Pose2d leftRedWingNote = new Pose2d((1.14 * merrickFieldConstant), ((fieldHeight/2)-(2*(.57 * merrickFieldConstant))), new Rotation2d(0));
    public static Pose2d middleRedWingNote = new Pose2d((1.14 * merrickFieldConstant), ((fieldHeight/2)-(.57 * merrickFieldConstant)), new Rotation2d(1));
    public static Pose2d rightRedWingNote = new Pose2d((1.14 * merrickFieldConstant), ((fieldHeight/2)), new Rotation2d(0));

    // :> Going from left to right
    public static Pose2d middleFieldNote1 = new Pose2d((fieldWidth/2), (.2964*merrickFieldConstant), new Rotation2d(0));
    public static Pose2d middleFieldNote2 = new Pose2d((fieldWidth/2), ((.2964+.66)*merrickFieldConstant), new Rotation2d(0));
    public static Pose2d middleFieldNote3 = new Pose2d((fieldWidth/2), ((.2964+(2*.66))*merrickFieldConstant), new Rotation2d(0));
    public static Pose2d middleFieldNote4 = new Pose2d((fieldWidth/2), ((.2964+(3*.66))*merrickFieldConstant), new Rotation2d(0));
    public static Pose2d middleFieldNote5 = new Pose2d((fieldWidth/2), ((.2964+(4*.66))*merrickFieldConstant), new Rotation2d(0));
    // :> I swear Merrick just pulls numbers out of a hat
    // :> Merrick should really become a mathematician with all the random constants and numbers he pulls out of the ether
    // :> I really hope I typed these all in correctly or we are going to have a repeat of last years incident with Bryce


    // :> This is a reference for shooting, it is the center of the speaker relative to the field
    // :> Worth noting I took all these measurements myself using the field onshape layout
    public static Pose2d blueSpeakerCenterReference = new Pose2d((0.4572 / 2), ((fieldHeight - 2.035) - (1.05 / 2)), new Rotation2d(0));
    public static Pose2d redSpeakerCenterReference = new Pose2d((fieldWidth - (0.4572 / 2)), ((fieldHeight - 2.035) - (1.05 / 2)), new Rotation2d(0));

    // :> Worth noting (teehee) these are going to the be used for mainly alligning to the amp as we score into it
    public static Pose2d blueAmp = new Pose2d(((fieldWidth/2)+(2.50550*merrickFieldConstant)), 0, new Rotation2d(0));
    public static Pose2d redAmp = new Pose2d(((fieldWidth/2)-(2.50550*merrickFieldConstant)), 0, new Rotation2d(0));

    public static Pose2d blueStagePosition1 = new Pose2d(4.481,  (fieldHeight-3.339), new Rotation2d(0));
    public static Pose2d blueStagePosition2 = new Pose2d(4.553, (fieldHeight-4.937), new Rotation2d(0));
    public static Pose2d blueStagePosition3 = new Pose2d(5.762, (fieldHeight-4.103), new Rotation2d(0));


    public static Pose2d redStagePosition1 = new Pose2d(12.16, (fieldHeight-3.339), new Rotation2d(0));
    public static Pose2d redStagePosition2 = new Pose2d(12.065, (fieldHeight-4.937), new Rotation2d(0));
    public static Pose2d redStagePosition3 = new Pose2d(10.78, (fieldHeight-4.1), new Rotation2d(0));

    public static Pose2d[] noteBluePositions = new Pose2d[]{leftBlueWingNote, middleBlueWingNote, rightBlueWingNote, middleFieldNote1, middleFieldNote2, middleFieldNote3, middleFieldNote4, middleFieldNote5};
    public static Pose2d[] noteRedPositions = new Pose2d[]{leftRedWingNote, middleRedWingNote, rightRedWingNote, middleFieldNote1, middleFieldNote2, middleFieldNote3, middleFieldNote4, middleFieldNote5};
    public static Pose2d[] stageRedPositions = new Pose2d[]{redStagePosition1, redStagePosition2, redStagePosition3};  
    public static Pose2d[] stageBluePositions = new Pose2d[]{blueStagePosition1, blueStagePosition2, blueStagePosition3}; 
  }

  public static class IntakeConstants {
    public static final int IntakeMotorID = 12;
    public static final int fourBarMotor = 11;

    public static final double intakeConversionFactor = 1/15;
    // :> Conversion factor from the motor to the gearbox
    public static final double fourBarConversionFactor = 1/64;

    // :> TODO: These might have to be negative depending on how the encoder sees it so these may need to be changed
    // ss These are in Rotations, NOT DEGREES OR RADIANS
    public static final double fourBarUndeployedSetPoint = 0;
    public static final double fourBarHalfDeployedSetPoint = 0.128;
    public static final double fourBarFullyDeployedSetPoint = 0.256;

    // ss the bounds for the getFourBarAtPosition() function as a ratio
    public static final double lowerBound = 0.99;
    public static final double upperBound = 1.01;
    // ss intake velocity (negative to intake)
    public static final double intakeV = -1;
    /*
    public static final class IntakePIDs {
      // ss todo: Tune the PIDs
      public static final double kP = 0.005;
      public static final double kI = 0.00;
      public static final double kD = 0;
      public static final double kFF = 0.00;
      public static final double kV = -1;
    }
    */

    public static final class FourBarPIDs {
      // :> todo: Change these to actual PID values when we get the robot
      public static final double fourBarP = .5;
      public static final double fourBarI = .001;
      public static final double fourBarD = .002;
      public static final double fourBarFF = 0;
    }

    // ss I think these are the ids for the limit switches? I just put some values in to stop some errors
    public static final class IntakeLimitSwitches {
      public static final int limitSwitchMax = 4;
      public static final int limitSwitchMin = 5;
    }
  }

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

      // ££ Sets how many rotations the motors need to complete to be fully extended
      public static final double motorPositionLimit = 50.0;

      public static final class IDs {
        // ££ Climber ids
        public static final int kMotorOne = 21;
        public static final int kMotorTwo = 22;

        // ££ Limit switch ids
        public static final int kSwitchOne = 2;
        public static final int kSwitchTwo = 3;
      }

      public static final class MotorSpeeds {
        // ££ Sets the speeds of the motors on the rise and fall
        public static final double kRiseSpeed = 0.5;
        public static final double kInitialFallSpeed = -0.5;
      }

      public static final class ClimberPIDFF {
        // ££ Sets the P, I, D, and FF values for the climber motors when they're pulling the robot up
        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kFF = 0.01;
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
      public static final IdleMode kTurningMotorIdleMode = IdleMode.kCoast;

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

        public static final double kTurningP = 0.3;
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
      public static final double kDrivingSpeedDamper = 2.5; // :3 meters per second
      public static final double kSlowDrivingSpeedDamper = 0.8;

      // :> Speed Damper for the rotation of the robot
      public static final double kAngularSpeedDamper = 1.4 * Math.PI; // :3 radians per second

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
      public static final double kMaxDrivingAcceleration = 12;

      // :3 if the gyro is reversed
      public static final boolean kGyroReversed = false;

      // :3 spark max ids
      public static final class IDs {

        // :3 driving ids
        public static final int kFrontLeftDrivingCanId = 4;
        public static final int kRearLeftDrivingCanId = 6;
        public static final int kFrontRightDrivingCanId = 2;
        public static final int kRearRightDrivingCanId = 8;

        // :3 turning ids
        public static final int kFrontLeftTurningCanId = 3;
        public static final int kRearLeftTurningCanId = 5;
        public static final int kFrontRightTurningCanId = 1;
        public static final int kRearRightTurningCanId = 7;
      }

      // :3 absolute encoder offsets (should be multiples of pi / 2
      // :3 if the encoders were zeroed properly in rev client)
      public static final class ModuleOffsets {
        public static final Rotation2d kFrontLeftOffset = Rotation2d.fromRadians(-Math.PI * 0.5);
        public static final Rotation2d kFrontRightOffset = Rotation2d.fromRadians(0);
        public static final Rotation2d kBackLeftOffset = Rotation2d.fromRadians(Math.PI);
        public static final Rotation2d kBackRightOffset = Rotation2d.fromRadians(Math.PI * 0.5);
      }

      // :3 things involving the physical setup of the chassis
      public static final class ChassisKinematics {

        // :3 distance between centers of right and left wheels on robot
        public static final double kRobotWidth = Units.inchesToMeters(26);
        // :3 distance between front and back wheels on robot
        public static final double kRobotLength = Units.inchesToMeters(26);

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
        public static final double kMaxAngularAccelerationRadians = 2.5;

        public static final double kTuringP = 2.5;
        public static final double kTurningI = 0.1;
        public static final double kTurningD = 0.08;

        public static final TrapezoidProfile.Constraints kTurningConfig =
          new TrapezoidProfile.Constraints(kMaxAngularVelocityRadians, kMaxAngularAccelerationRadians);

        public static final ProfiledPIDController kTurningPID =
          new ProfiledPIDController(kTuringP, kTurningI, kTurningD, kTurningConfig);

        // :3 driving setpoint stuff
        public static final double kMaxDrivingVelocity = 3.5;
        public static final double kMaxDrivingAcceleration = 2;

        public static final double kTrajectoryP = 0.2;
        public static final double kTrajectoryI = 0.005;
        public static final double kTrajectoryD = 0.01;

        public static final TrajectoryConfig kTrajectoryConfig =
          new TrajectoryConfig(kMaxDrivingVelocity, kMaxDrivingAcceleration);
      }
    }
  }

  public static final class LED {
    //&& These are the constants for the LEDSubsystem.
    //&& They can be moved to JSON later, I just don't know how to do that yet.

    //&& todo: Set the correct value for the PWM port, because I don't know if 0 is correct.
    public static final int pwmPort = 0;
  }
  public static final class PhotonVision {
    //TODO replace placeholders (maybe done H!)
    public static final String cameraName = "Microsoft_LifeCam_HD-3000";

    public static final String fieldLayoutPath = ""; // H! this isn't actually used; instead a built in file is used.
    public static final Pose3d cameraPosition =
      new Pose3d(new Translation3d(RobotConstants.frameWidth/2. - Units.inchesToMeters(2.5), 0, Units.inchesToMeters(16.75)), new Rotation3d());
    public static final Transform3d robotToCamera = new Transform3d(new Pose3d(), cameraPosition);
  }

  public static final class Plate {
    public static final int motorID = -999; //todo fill in number
    public static final double converstionFactor = 1/45.; // todo fill in number
    public static final double allowablePosDif = 0.05;
    public static final double allowableVelDif = 0.05;

    public static final double manualSpeedFactor = 0.1;

    public static final class PIDValues {
      public static final double p = 0.01;
      public static final double i = 0;
      public static final double d = 0;
      public static final double ff = 0.01;
    }

    public static final class Positions {
      public static final double stowed = 0.0; //TODO fill in numbers
      public static final double amp = 0.0;
      public static final double speaker = 0.0;
    }
  }

  public static final class ShooterConstants {
    
    public static final int leftMotorID = 0;
    public static final int rightMotorID = 0;

    

    // && The target speeds for the speaker and the amp (range will be based on these)
    // && todo: put in actual, reasonable numbers for all of the things below
    // H! the above should be done, but the numbers are currently untested
    public static final double ampShootSpeed = 5;
    public static final double speakerShootSpeed = 60;
    public static final double stopShootSpeed = 0.0;
    
    // && The range of shooter speeds that are "acceptable" on either side of the target speed 
    public static final double ampSpeedRange = 1;
    public static final double speakerSpeedRange = 1;
    public static final double stopSpeedRange = 1;
  }

  public static final class ColorSensor {
    // H! Larger is closer
    public static final int emptyDistance = 600;//H! todo tune these
    public static final int filledDistance = 800;
  }
 
}
