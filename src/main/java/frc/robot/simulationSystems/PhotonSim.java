package frc.robot.simulationSystems;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.CameraConstants.PiCamera;
import frc.robot.subsystems.PhotonCameraSystem;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

public class PhotonSim {
  private static PhotonCameraSim cameraSim;
  private static VisionSystemSim visionSim;

  public static void setupSim() {
    cameraSim = new PhotonCameraSim(PhotonCameraSystem.getCamera());
    visionSim = new VisionSystemSim(cameraSim.getCamera().getName());
    visionSim.addCamera(cameraSim, PiCamera.robotToCam);
    visionSim.addAprilTags(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
  }

  public static void update(Pose2d robotPose) {
    if (visionSim == null) return;
    visionSim.update(robotPose);
  }

  public static PhotonCameraSim getCameraSim() {
    return cameraSim;
  }

  public static void moveCameraSim(Pose2d pose) {
    visionSim.update(pose);
  }
}
