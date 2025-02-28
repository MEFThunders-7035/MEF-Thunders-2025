package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.CameraConstants.PiCamera;
import frc.robot.Constants.CameraConstants.TestCamera;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision implements AutoCloseable {
  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  private final PhotonCamera testCamera;
  private final PhotonPoseEstimator testCameraEstimator;

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;

  private final Watchdog watchdog = new Watchdog(0.01, () -> printEpochs());

  private final Map<PhotonCamera, PhotonPoseEstimator> cameraEstimators;
  private Matrix<N3, N1> curStdDevs;

  private Supplier<Pose2d> poseSupplier;

  // Simulation
  private PhotonCameraSim cameraSim;
  private PhotonCameraSim testCameraSim;
  private VisionSystemSim visionSim;

  /**
   * @param getActualPose Current odometry predicted pose
   */
  public Vision(Supplier<Pose2d> getActualPose) {
    camera = new PhotonCamera(PiCamera.cameraName);
    testCamera = new PhotonCamera(TestCamera.cameraName);

    photonEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PiCamera.robotToCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    testCameraEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, TestCamera.robotToCam);
    testCameraEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    cameraEstimators = Map.of(camera, photonEstimator, testCamera, testCameraEstimator);

    poseSupplier = getActualPose;

    // ----- Simulation
    if (RobotBase.isSimulation()) {
      initSimCameras();
    }
  }

  private void printEpochs() {
    watchdog.printEpochs();
  }

  private void initSimCameras() {
    // Create the vision system simulation which handles cameras and targets on the field.
    visionSim = new VisionSystemSim("main");
    // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
    visionSim.addAprilTags(fieldLayout);
    // Create simulated camera properties. These can be set to mimic your actual camera.
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(15);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setLatencyStdDevMs(40);
    // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
    // targets.
    cameraSim = new PhotonCameraSim(camera, cameraProp);
    var testCameraProp = new SimCameraProperties();
    testCameraProp.setCalibration(1280, 960, Rotation2d.fromDegrees(90));
    testCameraProp.setCalibError(0.55, 0.24);
    testCameraProp.setFPS(15);
    testCameraProp.setAvgLatencyMs(250);
    testCameraProp.setLatencyStdDevMs(20);

    testCameraSim = new PhotonCameraSim(testCamera, testCameraProp);
    // Add the simulated camera to view the targets on this simulated field.
    visionSim.addCamera(cameraSim, PiCamera.robotToCam);
    visionSim.addCamera(testCameraSim, TestCamera.robotToCam);

    cameraSim.enableDrawWireframe(false);
    testCameraSim.enableDrawWireframe(false);
  }

  @Override
  public void close() {
    camera.close();
    testCamera.close();

    cameraSim.close();
    testCameraSim.close();
    visionSim.clearVisionTargets();
    visionSim.clearCameras();
    getSimDebugField().close();
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
   * {@link getEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return getEstimatedGlobalPose(poseSupplier.get());
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
   * {@link getEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   * @param currentPose the robot pose estimated by odometry, if there are multiple results, will
   *     select the one closer to this pose.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d currentPose) {
    watchdog.reset();
    var listOfEstimates = getAllEstimatedRobotPoses();
    watchdog.addEpoch("GetAllEstimates");

    if (listOfEstimates.isEmpty()) return Optional.empty();

    watchdog.addEpoch("CheckEmpty");
    if (listOfEstimates.size() == 1) return filter(listOfEstimates.get(0), currentPose);
    watchdog.addEpoch("Filter");

    // just these lines are enough to make the roborio KYS...
    Optional<EstimatedRobotPose> closestEstimate =
        filter(
            listOfEstimates.stream()
                .min(
                    (pose1, pose2) ->
                        (int)
                            (getDifferenceToPose(pose1, currentPose)
                                - getDifferenceToPose(pose2, currentPose)))
                .get(),
            currentPose);

    watchdog.addEpoch("ClosestEstimate");
    watchdog.disable();
    return closestEstimate;
  }

  private double getDifferenceToPose(EstimatedRobotPose estimatedPose, Pose2d odometryPose) {
    return estimatedPose
        .estimatedPose
        .getRotation()
        .toRotation2d()
        .minus(odometryPose.getRotation())
        .getDegrees();
  }

  private Optional<EstimatedRobotPose> filter(
      EstimatedRobotPose estimatedPoseOpt, Pose2d odometryPose) {
    var estimatedPose = estimatedPoseOpt.estimatedPose;
    var isRotationTooDifferent =
        estimatedPose
            .getRotation()
            .toRotation2d()
            .minus(odometryPose.getRotation())
            .getMeasure()
            .lt(Degrees.of(5));
    return isRotationTooDifferent ? Optional.of(estimatedPoseOpt) : Optional.empty();
  }

  public List<EstimatedRobotPose> getAllEstimatedRobotPoses() {
    return Stream.of(
            getEstimatedGlobalPoseFromCamera(camera), getEstimatedGlobalPoseFromCamera(testCamera))
        .filter(Optional::isPresent)
        .map(Optional::get)
        .toList();
  }

  private Optional<EstimatedRobotPose> getEstimatedGlobalPoseFromCamera(PhotonCamera cameraToUse) {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var result : cameraToUse.getAllUnreadResults()) {
      cameraEstimators.get(cameraToUse).update(result);
      visionEst = photonEstimator.update(result);
      updateEstimationStdDevs(visionEst, result.getTargets());

      if (RobotBase.isSimulation()) {
        visionEst.ifPresentOrElse(
            est ->
                getSimDebugField()
                    .getObject("VisionEstimation")
                    .setPose(est.estimatedPose.toPose2d()),
            () -> getSimDebugField().getObject("VisionEstimation").setPoses());
      }
    }
    return visionEst;
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = PiCamera.kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = PiCamera.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = PiCamera.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = PiCamera.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  // ----- Simulation

  public void simulationPeriodic() {
    simulationPeriodic(poseSupplier.get());
  }

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
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
