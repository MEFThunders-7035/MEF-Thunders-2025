package subsystem_tests.drive_subsystem_tests.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.SwerveModuleConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTestUtils {
  public static ChassisSpeeds getDesiredChassisSpeeds(DriveSubsystem driveSubsystem) {
    return SwerveModuleConstants.kDriveKinematics.toChassisSpeeds(
        driveSubsystem.getModuleDesiredStates());
  }

  public static ChassisSpeeds driveToChassisSpeeds(
      double x, double y, double rot, Rotation2d gyroAngle) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        x * DriveConstants.kMaxSpeedMetersPerSecond,
        y * DriveConstants.kMaxSpeedMetersPerSecond,
        rot * DriveConstants.kMaxAngularSpeed,
        gyroAngle);
  }

  public static ChassisSpeeds driveToChassisSpeeds(double x, double y, double rot) {
    return driveToChassisSpeeds(x, y, rot, new Rotation2d());
  }

  public static void checkIfEqual(ChassisSpeeds shouldBe, ChassisSpeeds currentlyIs) {
    assertEquals(shouldBe.vxMetersPerSecond, currentlyIs.vxMetersPerSecond, 0.01);
    assertEquals(shouldBe.vyMetersPerSecond, currentlyIs.vyMetersPerSecond, 0.01);
    assertEquals(shouldBe.omegaRadiansPerSecond, currentlyIs.omegaRadiansPerSecond, 0.01);
  }

  public static void checkIfNotEqual(ChassisSpeeds shouldBe, ChassisSpeeds currentlyIs) {
    assertNotEquals(shouldBe.vxMetersPerSecond, currentlyIs.vxMetersPerSecond, 0.01);
    assertNotEquals(shouldBe.vyMetersPerSecond, currentlyIs.vyMetersPerSecond, 0.01);
    assertNotEquals(shouldBe.omegaRadiansPerSecond, currentlyIs.omegaRadiansPerSecond, 0.01);
  }
}
