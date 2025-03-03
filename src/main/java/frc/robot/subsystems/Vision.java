package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.CameraConstants.Camera;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;

public class Vision implements AutoCloseable {
  Alert alert =
      new Alert(
          "Cannot find ID of april tag that was detected in list, VERY BAD, RETURNING OPTIONAL.EMPTY",
          AlertType.kError);

  Tracer tracer = new Tracer();

  VisionSystemSim visionSim;
  Supplier<Pose2d> poseSupplier;
  List<PhotonPoseCamera> cameras;
  PhotonPoseCamera bestCamera;
  double bestPoseConfidence;

  public Vision(Supplier<Pose2d> odometryPoseSupplier, List<PhotonPoseCamera> cameras) {
    if (cameras.isEmpty()) {
      throw new IllegalArgumentException("Vision must have at least one camera");
    }

    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(CameraConstants.kFieldLayout);
    poseSupplier = odometryPoseSupplier;
    bestCamera = cameras.get(0);
    this.cameras = cameras;

    for (PhotonPoseCamera camera : cameras) {
      camera.addCameraToSimField(visionSim);
    }
  }

  public static Vision fromCameraConstants(
      Supplier<Pose2d> odometryPoseSupplier, List<Camera> cameraConstants) {
    List<PhotonPoseCamera> cameras = cameraConstants.stream().map(PhotonPoseCamera::new).toList();
    return new Vision(odometryPoseSupplier, cameras);
  }

  @Override
  public void close() {
    visionSim.clearCameras();
    visionSim.clearAprilTags();
    visionSim.clearVisionTargets();

    for (PhotonPoseCamera camera : cameras) {
      camera.close();
    }
  }

  /**
   * The confidence value of the found estimated pose, if it has been found.
   *
   * @return a confidence value between 0 and 1, closer to 1 means more confident.
   */
  public double getConfidence() {
    return bestPoseConfidence;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return getEstimatedGlobalPose(poseSupplier.get());
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d odometryPose) {
    EstimatedRobotPose bestPose = null;
    tracer.resetTimer();
    tracer.clearEpochs();

    // Find best pose, confidence, and camera
    for (PhotonPoseCamera camera : cameras) {
      var pose = camera.getEstimatedGlobalPose();
      tracer.addEpoch("getEstimatedGlobalPose for camera: " + camera.getName());

      if (pose.isEmpty()) {
        continue;
      }

      var detectedPose = pose.get();
      var confidence = getPoseConfidence(detectedPose, odometryPose);
      tracer.addEpoch("getPoseConfidence for camera: " + camera.getName());

      if (bestPose == null || confidence > bestPoseConfidence) {
        bestPose = detectedPose;
        bestPoseConfidence = confidence;
        bestCamera = camera;
      }
    }

    if (bestPose == null) {
      return Optional.empty();
    }

    // If only one tag was used, we can use the difference between the rotations of both positions
    // to calculate the actual correct estimated position instead of the wrong one
    if (bestPose.targetsUsed.size() < 2) {
      var usedAprilTag = bestPose.targetsUsed.get(0);
      var currentTagPose = CameraConstants.kFieldLayout.getTagPose(usedAprilTag.getFiducialId());
      if (currentTagPose.isEmpty()) {
        return Optional.empty();
      }
      // TODO: Find the error through the difference between the rotation of the robot and the
      // rotation of the tag, and then correct the estimated pose using some trigonometry
      var robotRotation = odometryPose.getRotation();
      var estimatedRotation = bestPose.estimatedPose.getRotation();
      var rotationDifference = robotRotation.minus(estimatedRotation.toRotation2d());
    }

    return Optional.of(bestPose);
  }

  /**
   * Get the confidence of a pose based on the distance to the odometry pose.
   *
   * @param pose the estimated pose to calculate the confidence of
   * @param odometryPose the position that is currently guessed by odometry.
   * @return a confidence value between 0 and 1, closer to 1 means more confident.
   */
  private static double getPoseConfidence(EstimatedRobotPose pose, Pose2d odometryPose) {
    if (pose.targetsUsed.size() < 2 || pose.strategy != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
      return 0;
    }

    var rotationDifference =
        Math.abs(pose.estimatedPose.getRotation().getZ() - odometryPose.getRotation().getRadians());
    // If the robot is rotated more than 8.5 degrees from the vision system, the vision system is
    // not reliable.
    if (rotationDifference > 0.15) { // 0.15 radians ~= 8.5 degrees
      return 0;
    }

    var distanceToOdometry =
        Math.abs(
            pose.estimatedPose
                .toPose2d()
                .getTranslation()
                .getDistance(odometryPose.getTranslation()));

    return 1.0 / (1.0 + distanceToOdometry + 2 * rotationDifference);
  }

  // ----- Simulation
  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
    SmartDashboard.putString("Best Camera", bestCamera.getName());
    tracer.printEpochs(trace -> SmartDashboard.putString("Vision Trace", trace));
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (RobotBase.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!RobotBase.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
