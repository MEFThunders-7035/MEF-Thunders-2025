package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.CameraConstants.PiCamera;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * This class is used to interface with the PhotonCamera and PhotonPoseEstimator classes. It is used
 * to get the robot's pose on the field using only AprilTags.
 */
public final class PhotonCameraSystem {
  private static PhotonCamera camera = new PhotonCamera(PiCamera.cameraName);
  private static PhotonPoseEstimator photonPoseEstimator = getPhotonPoseEstimator();
  private static AprilTagFieldLayout fieldLayout;

  private static int loadTry = 0;

  private static PhotonPoseEstimator getPhotonPoseEstimator() {
    // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    // Create pose estimator
    photonPoseEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PiCamera.robotToCam);
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    System.out.println("Loaded PhotonPoseEstimator");
    return photonPoseEstimator;
  }

  /**
   * You should never need this, but is here for simulating the camera.
   *
   * @return the camera object the system is using.
   */
  public static PhotonCamera getCamera() {
    return camera;
  }

  public static AprilTagFieldLayout getFieldLayout() {
    return fieldLayout;
  }

  /**
   * This is used for a Target that you know the height of. If the target is not found, it will
   * return empty. It also won't check if the target is actually the target you are looking for. It
   * will just return the distance to the first target it sees.
   *
   * @param targetHeightMeters The height of the target in meters.
   * @return the distance to the target in meters.
   */
  public static Optional<Double> getDistanceToTarget(double targetHeightMeters) {
    var pitch = getPitch();
    if (pitch == 0) return Optional.empty();
    return Optional.of(
        PhotonUtils.calculateDistanceToTargetMeters(
            PiCamera.kCameraHeight, targetHeightMeters, pitch, PiCamera.kCameraPitchRadians));
  }

  private static Alert driverModeAlert =
      new Alert("Driver Mode Was ON! Turning it off...", Alert.AlertType.kWarning);

  public static PhotonPipelineResult getLatestResult() {
    if (camera.getDriverMode()) {
      driverModeAlert.set(true);
      camera.setDriverMode(false);
    } else {
      driverModeAlert.set(false);
    }
    return camera.getLatestResult();
  }

  /**
   * Returns the pitch of the target according to the camera.
   *
   * @return the difference between the middle of the camera and the target In Terms of Pitch. If no
   *     target is found, it will return 0 (I have no clue what the units are)
   */
  public static double getPitch() {
    var latestResult = getLatestResult();
    if (latestResult.hasTargets()) {
      return latestResult.getBestTarget().getPitch();
    }
    return 0;
  }

  /**
   * Returns the Yaw of the target according to the camera.
   *
   * @return the difference between the middle of the camera and the target In terms of Yaw. If no
   *     target is found, it will return 0. (I have no clue what the units are)
   */
  public static double getYaw() {
    var latestResult = getLatestResult();
    if (latestResult.hasTargets()) {
      return latestResult.getBestTarget().getYaw();
    }
    return 0;
  }

  /**
   * Returns the Area of the target according to the camera.
   *
   * @return the area percentage (0 to 100) of the camera fov.
   */
  public static double getArea() {
    var latestResult = getLatestResult();
    if (latestResult.hasTargets()) {
      return latestResult.getBestTarget().getArea();
    }
    return 0;
  }

  /**
   * @return The current id of the best april tag being tracked. If no tag is being tracked, it will
   *     return -1.
   */
  public static int getCurrentAprilTagID() {
    var latestResult = getLatestResult();
    if (latestResult.hasTargets()) {
      return latestResult.getBestTarget().getFiducialId();
    }
    return -1;
  }

  /**
   * @return the current ids of each aprilTag being tracked. If there are no aprilTags is being
   *     tracked, it will return an empty array.
   */
  public static List<Integer> getTrackedTargetsIDs() {
    List<Integer> ids = new ArrayList<>();
    var latestResult = getLatestResult();
    // If there are no targets, return an empty array.
    if (!latestResult.hasTargets()) return ids;
    // Get the ids of each target.
    var targets = latestResult.getTargets();
    for (var target : targets) {
      ids.add(target.getFiducialId());
    }
    return ids;
  }

  public static List<PhotonTrackedTarget> getTrackedTargets() {
    return camera.getLatestResult().getTargets();
  }

  /**
   * Finds the PhotonTrackedTarget with the given id if seen by the camera.
   *
   * @param id The id of the aprilTag you want to find.
   * @return The PhotonTrackedTarget with the given id. If no target is found, it will return null.
   */
  public static Optional<PhotonTrackedTarget> getAprilTagWithID(int id) {
    var targets = getTrackedTargets();
    for (var target : targets) {
      if (target.getFiducialId() == id) {
        return Optional.of(target);
      }
    }
    return Optional.empty();
  }

  /**
   * Lets you Select the LED mode of the camera.
   *
   * @param state The state of the LED that you want.
   */
  public static void setLed(VisionLEDMode state) {
    camera.setLED(state);
  }

  /**
   * Returns the robot's pose on the field If found. If not found, it will return empty.
   *
   * @param prevEstimatedRobotPose .
   * @return The new {@link EstimatedRobotPose} To get {@link Pose2d} use {@code
   *     EstimatedRobotPose.get().estimatedPose.toPose2d()}.
   */
  public static Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (photonPoseEstimator == null) {
      if (loadTry > 5) {
        photonPoseEstimator = getPhotonPoseEstimator();
        loadTry = 0;
        return getEstimatedGlobalPose(prevEstimatedRobotPose);
      }
      DriverStation.reportError("Pose Estimator Couldn't be loaded", false);
      loadTry += 1;
      // The field layout failed to load, so we cannot estimate poses.
      return Optional.empty();
    }
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update(getLatestResult());
  }

  /**
   * Returns the robot's pose on the field If found. If not found, it will return empty.
   *
   * @param prevEstimatedRobotPose .
   * @return The new {@link EstimatedRobotPose} To get {@link Pose2d} use {@code
   *     EstimatedRobotPose.get().estimatedPose.toPose2d()}.
   */
  public static Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    if (photonPoseEstimator == null) {
      if (loadTry > 5) {
        photonPoseEstimator = getPhotonPoseEstimator();
        loadTry = 0;
        return getEstimatedGlobalPose();
      }
      DriverStation.reportError("Pose Estimator Offline", false);
      loadTry += 1;
      // The field layout failed to load, so we cannot estimate poses.
      return Optional.empty();
    }
    if (camera.getDriverMode()) {
      camera.setDriverMode(false);
      System.out.println("Camera Was in driver mode, switched to off...");
    }
    return photonPoseEstimator.update(getLatestResult());
  }
}
