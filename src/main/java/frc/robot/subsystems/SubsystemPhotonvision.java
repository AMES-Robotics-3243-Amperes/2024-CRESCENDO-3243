package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DataManager;

import static frc.robot.Constants.PhotonVision.*;

import java.io.IOException;
import java.util.Optional;

/**
 * A subsystem for using a camera to find the robot's position. This
 * uses a software known as "PhotonVision", and the raspberry pi as a 
 * coprocessor
 * 
 * @author H!
 */
public class SubsystemPhotonvision extends SubsystemBase {
    
  protected PhotonCamera camera;
  protected static AprilTagFieldLayout fieldLayout;
  protected PhotonPoseEstimator poseEstimator;

  /** Creates a new SubsystemPhotonVision. */
  public SubsystemPhotonvision() throws IOException {
    camera = new PhotonCamera(cameraName);
    fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void periodic() {
    
    Optional<EstimatedRobotPose> poseLatestOptional = poseEstimator.update();
    PhotonPipelineResult pipelineResult = camera.getLatestResult();

    if (poseLatestOptional.isPresent() && pipelineResult.targets.size() != 0) {
      // Get the ambiguity of the best target and divide it by the number of targets to account for the fact 
      // multiple targets give us better position data
      double adjustedAmbiguity = pipelineResult.getBestTarget().getPoseAmbiguity() / pipelineResult.targets.size();
      EstimatedRobotPose poseLatest = poseLatestOptional.get();

      DataManager.currentRobotPose.updateWithVision(poseLatest.estimatedPose, adjustedAmbiguity);
    } else {
      DataManager.currentRobotPose.updateWithVision(null, 10000);
    }
}
}
