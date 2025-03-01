package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.photonvision.simulation.VisionSystemSim;

public class Vision {
  VisionSystemSim visionSim;

  public Vision() {
    visionSim = new VisionSystemSim("main");
  }

  // ----- Simulation
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
