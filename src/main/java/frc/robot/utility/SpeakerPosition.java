package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public enum SpeakerPosition {
    ampside(
      new Pose2d(Constants.FieldConstants.fieldWidth - 1.69, 6.67 + Units.inchesToMeters(22.25), Rotation2d.fromDegrees(-(225-180))),
      new Pose2d(1.6, 6.55 + Units.inchesToMeters(22.25), Rotation2d.fromDegrees(225))
    ),
    center(
      new Pose2d(Constants.FieldConstants.fieldWidth - 2.22, 5.1, Rotation2d.fromDegrees(0)),
      new Pose2d(2, 5.1 + Units.inchesToMeters(22.25), Rotation2d.fromDegrees(180))
    ),
    sourceside(
      new Pose2d(Constants.FieldConstants.fieldWidth - 1.69, 3.95 + Units.inchesToMeters(22.25), Rotation2d.fromDegrees((-(140.4-180)))),
      new Pose2d(1.6, 4.2 + Units.inchesToMeters(22.25), Rotation2d.fromDegrees(140.4))
    );

    Pose2d redPose;
    Pose2d bluePose;

    SpeakerPosition(Pose2d redPose, Pose2d bluePose) {
      this.redPose = redPose;
      this.bluePose = bluePose;
    }

    public Pose2d getPose() {
      return DriverStation.getAlliance().get() == Alliance.Red ? this.redPose : this.bluePose;
    }
  }