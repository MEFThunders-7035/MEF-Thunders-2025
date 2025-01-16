package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.I2C;
import java.util.Optional;

public final class Constants {
  private Constants() {} // Prevents instantiation, as this is a utility class

  public static final class UtilConstants {
    public static final int kRoborioDIOCount = 9;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = 0.71; // 71 cm between left and right wheels on robot
    public static final double kWheelBase = 0.79; // 79 cm between front and back wheels on robot

    public static final class MotorConstants {
      // Spark MAX CAN IDs
      public static final int kFrontLeftDrivingCanID = 1;
      public static final int kFrontLeftTurningCanID = 2;

      public static final int kFrontRightDrivingCanID = 3;
      public static final int kFrontRightTurningCanID = 4;

      public static final int kRearLeftDrivingCanID = 5;
      public static final int kRearLeftTurningCanID = 6;

      public static final int kRearRightDrivingCanID = 7;
      public static final int kRearRightTurningCanID = 8;
    }

    public static final class NeoMotorConstants {
      public static final double kFreeSpeedRpm = 5676;
    }

    public static final class SwerveModuleConstants {
      // Angular offsets of the modules relative to the chassis in radians
      public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
      public static final double kFrontRightChassisAngularOffset = 0;
      public static final double kBackLeftChassisAngularOffset = Math.PI;
      public static final double kBackRightChassisAngularOffset = Math.PI / 2;
      public static final SwerveDriveKinematics kDriveKinematics =
          new SwerveDriveKinematics(
              new Translation2d(kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    }
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in
    // a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps =
        DriveConstants.NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on
    // the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput =
        kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 30; // amps
  }

  public static final class OIConstants {
    public static final double kDriveDeadband = 0.02;
    public static final double kDriveSensitivity = 0.5; // The rest will be added by "boost"
    public static final double kMaxBoost = 1.0;
    public static final double kMinBoost = -0.7;
    public static final double kDriveMaxOutput = 1.0;
  }

  public static final class IntakeConstants {
    public static final double kIntakeSpeed = 1;
    public static final double kGroundIntakeRunSpeed = kIntakeSpeed; // used while intaking
    public static final double kArmIntakeRunSpeed = kIntakeSpeed; // only while intaking
    public static final double kPushToShooterSpeed =
        kIntakeSpeed; // only while pushing to shooter, which uses arm only

    public static final int kArmIntakeMotorCanID = 9;
    public static final int kGroundIntakeMotorCanID = 13;
    public static final int kArmMotorCanID = 10;
    public static final int kArmFollowerMotorCanID = 11;

    public static final IdleMode kArmMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kIntakeMotorIdleMode = IdleMode.kBrake;

    public static final class ColorSensorConstants {
      public static final I2C.Port kColorSensorPort = I2C.Port.kMXP; // Connected to the NavX MXP
    }

    public static final int kArmEncoderCPR = 4096; // Trough Bore
    public static final double kArmEncoderGearAmount = 1; // Directly Connected
    public static final double kArmEncoderPositionFactor = 1 / (kArmEncoderGearAmount);
    public static final int kSmartCurrentLimit = 40;

    public static final class ArmPIDConstants {
      public static final double kP = 3; // 3
      public static final double kI = 0.02; // 0.02
      public static final double kD = 8; // 4
      public static final double kFF = 0.0; // 0

      // Feedforward gains
      // ! TODO: TUNE
      public static final double kS = 0.0;
      public static final double kG = 2;
      public static final double kV = 0.0;

      public static final double kAllowedError = 0.01;
    }
  }

  public static final class ArmConstants {
    public static final double AMP_POSITION = 0.5;

    public static final class ArmPIDConstants {
      public static final double kP = 3; // 3
      public static final double kI = 0.02; // 0.02
      public static final double kD = 8; // 4
      public static final double kFF = 0.0; // 0

      // Feedforward gains
      // ! TODO: TUNE
      public static final double kS = 0.0;
      public static final double kG = 2;
      public static final double kV = 0.0;

      public static final double kAllowedError = 0.01;
    }
  }

  public static final class ShooterConstants {
    public static final int kShooterMotorPwmID = 0;

    public static final int kShooter1CanID = 11;
    public static final int kShooter2CanID = 12;

    public static final double kShooterSpeed = 1;
  }

  public static final class CameraConstants {
    public static final class PiCamera {
      public static final String cameraName = "piCamera";
      public static final double kCameraHeight = 0.225;
      public static final double kCameraYDistanceMeters = -0.15; // 15 cm back of the robot
      public static final double kCameraXDistanceMeters = 0.18; // ~18 cm away sideways
      public static final double kCameraPitchRadians = Math.toRadians(-30); // 30 degrees up
      public static final double kCameraYawRadians = Math.toRadians(180); // looking to the back
      public static final Transform3d robotToCam =
          new Transform3d(
              new Translation3d(kCameraXDistanceMeters, kCameraYDistanceMeters, kCameraHeight),
              new Rotation3d(0, kCameraPitchRadians, kCameraYawRadians));
    }
  }

  public static final class LEDConstants {
    public static final int kLedPin = 9;
    public static final int kLedCount = 82;
  }

  // yes, it leaks memory, but it's a one-time thing, so it'sss fine...
  @SuppressWarnings("resource")
  private static final Optional<RobotConfig> getRobotConfig() {
    try {
      return Optional.of(RobotConfig.fromGUISettings());
    } catch (Exception e) {
      new Alert("Failed to get Robot Config", AlertType.kError).set(true);
    }

    return Optional.empty();
  }

  public static final class AutoConstants {
    public static final Optional<RobotConfig> kRobotConfig = getRobotConfig();

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final class DrivePIDController {
      public static final double kP = 2.0;
      public static final double kI = 0;
      public static final double kD = 0.5;
    }

    public static final class RotationPIDController {
      public static final double kP = 0.5;
      public static final double kI = 0;
      public static final double kD = 0.2;
    }
  }

  public static final class MagicConstants {
    public static final class ArmQuadraticFunction {
      // Quadratic function constants of: ax^2 + bx + c
      public static final double kXSquared = 0.0404292;
      public static final double kX = -0.0407415;
      public static final double kConstant = 0.089446;
    }
  }
}
