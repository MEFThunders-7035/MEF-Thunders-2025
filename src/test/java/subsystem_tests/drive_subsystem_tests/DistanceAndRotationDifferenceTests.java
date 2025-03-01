package subsystem_tests.drive_subsystem_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Vision;
import frc.utils.ExtraFunctions;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DistanceAndRotationDifferenceTests extends DriveSubsystemTestBase {
  private static final double delta = 0.0001;

  @BeforeEach
  @Override
  public void setUp() {
    super.setUp();
  }

  @AfterEach
  @Override
  public void tearDown() {
    super.tearDown();
  }

  @Test
  void testRotationDifference() {
    // AprilTag positions
    var tag = Vision.fieldLayout.getTagPose(ExtraFunctions.getShooterAprilTagID()).orElseThrow();

    driveSubsystem.resetOdometry(new Pose2d(tag.getX() + 2, tag.getY() + 2, new Rotation2d(0)));

    // we should be 45 degrees off from the target
    assertEquals(-45, driveSubsystem.getRotationDifferenceToShooter().getDegrees(), delta);
  }

  @Test
  void testDistanceDifference() {
    // AprilTag positions
    var tag = Vision.fieldLayout.getTagPose(ExtraFunctions.getShooterAprilTagID()).orElseThrow();

    driveSubsystem.resetOdometry(new Pose2d(tag.getX() + 2, tag.getY() + 2, new Rotation2d(0)));

    // we should be 2.8 meters away from the target
    assertEquals(Math.hypot(2, 2), driveSubsystem.getDistanceToShooter(), delta);
  }
}
