package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CameraConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FieldConstants {
  // COPIED FROM:
  // https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/src/main/java/org/littletonrobotics/frc2025/FieldConstants.java
  public static final double fieldLength = CameraConstants.kFieldLayout.getFieldLength();
  public static final double fieldWidth = CameraConstants.kFieldLayout.getFieldWidth();
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line
  public static final double algaeDiameter = Units.inchesToMeters(16);

  public static class Reef {
    public static final double faceLength = Units.inchesToMeters(36.792600);
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.0);
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    // This is in terms of blue alliance
    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order

    // This is in terms of red alliance
    public static final Pose2d[] redCenterFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final List<Map<ReefLevel, Pose3d>> branchPositions =
        new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise
    public static final List<Map<ReefLevel, Pose2d>> branchPositions2d = new ArrayList<>();

    static {
      // Initialize faces
      centerFaces[0] = CameraConstants.kFieldLayout.getTagPose(18).get().toPose2d();
      centerFaces[1] = CameraConstants.kFieldLayout.getTagPose(19).get().toPose2d();
      centerFaces[2] = CameraConstants.kFieldLayout.getTagPose(20).get().toPose2d();
      centerFaces[3] = CameraConstants.kFieldLayout.getTagPose(21).get().toPose2d();
      centerFaces[4] = CameraConstants.kFieldLayout.getTagPose(22).get().toPose2d();
      centerFaces[5] = CameraConstants.kFieldLayout.getTagPose(17).get().toPose2d();

      redCenterFaces[0] = CameraConstants.kFieldLayout.getTagPose(7).get().toPose2d();
      redCenterFaces[1] = CameraConstants.kFieldLayout.getTagPose(8).get().toPose2d();
      redCenterFaces[2] = CameraConstants.kFieldLayout.getTagPose(9).get().toPose2d();
      redCenterFaces[3] = CameraConstants.kFieldLayout.getTagPose(10).get().toPose2d();
      redCenterFaces[4] = CameraConstants.kFieldLayout.getTagPose(11).get().toPose2d();
      redCenterFaces[5] = CameraConstants.kFieldLayout.getTagPose(6).get().toPose2d();

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefLevel, Pose3d> fillRight = new HashMap<>();
        Map<ReefLevel, Pose3d> fillLeft = new HashMap<>();
        Map<ReefLevel, Pose2d> fillRight2d = new HashMap<>();
        Map<ReefLevel, Pose2d> fillLeft2d = new HashMap<>();
        for (var level : ReefLevel.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180.0 - (60 * face)));
          double adjustX = Units.inchesToMeters(30.738);
          double adjustY = Units.inchesToMeters(6.469);

          var rightBranchPose =
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians()));
          var leftBranchPose =
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians()));

          fillRight.put(level, rightBranchPose);
          fillLeft.put(level, leftBranchPose);
          fillRight2d.put(level, rightBranchPose.toPose2d());
          fillLeft2d.put(level, leftBranchPose.toPose2d());
        }
        branchPositions.add(fillRight);
        branchPositions.add(fillLeft);
        branchPositions2d.add(fillRight2d);
        branchPositions2d.add(fillLeft2d);
      }
    }
  }

  public enum ReefLevel {
    L1(Units.inchesToMeters(25.0), 0),
    L2(Units.inchesToMeters(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
    L3(Units.inchesToMeters(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
    L4(Units.inchesToMeters(72), -90);

    ReefLevel(double height, double pitch) {
      this.height = height;
      this.pitch = pitch; // Degrees
    }

    public static ReefLevel fromLevel(int level) {
      return Arrays.stream(values())
          .filter(height -> height.ordinal() == level)
          .findFirst()
          .orElse(L4);
    }

    public final double height;
    public final double pitch;
  }
}
