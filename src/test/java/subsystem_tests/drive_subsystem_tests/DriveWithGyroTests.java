package subsystem_tests.drive_subsystem_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DriveConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveWithGyroTests extends DriveSubsystemTestBase {

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
  void testDriveForwardWithAngleFieldRelative() {
    driveSubsystem.setSimulatedGyroAngle(-90);
    assertEquals(
        Rotation2d.fromDegrees(-90), driveSubsystem.getRotation2d(), "gyro sim is broken again...");
    runCommand(driveSubsystem.drive(0.5, 0, 0, true, false));

    for (var state : driveSubsystem.getModuleDesiredStates()) {
      assertEquals(DriveConstants.kMaxSpeedMetersPerSecond * 0.5, state.speedMetersPerSecond, 0.01);
      assertEquals(Rotation2d.fromDegrees(90), state.angle);
    }
  }

  @Test
  void testDriveSidewaysWithAngleFieldRelative() {
    driveSubsystem.setSimulatedGyroAngle(-45);
    assertEquals(
        Rotation2d.fromDegrees(-45), driveSubsystem.getRotation2d(), "gyro sim is broken again...");
    runCommand(driveSubsystem.drive(0, 0.5, 0, true, false));

    for (var state : driveSubsystem.getModuleDesiredStates()) {
      assertEquals(DriveConstants.kMaxSpeedMetersPerSecond * 0.5, state.speedMetersPerSecond, 0.01);
      // 90 + 45 degrees
      assertEquals(Rotation2d.fromDegrees(90 + 45), state.angle);
    }
  }
}
