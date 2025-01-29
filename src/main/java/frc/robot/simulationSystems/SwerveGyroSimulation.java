package frc.robot.simulationSystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants.SwerveModuleConstants;

public class SwerveGyroSimulation {
  private Alert alert =
      new Alert(
          "Setting the angle of the gyro in a real robot is not allowed. This is a simulation only feature.",
          AlertType.kWarning);

  /** Main timer to control movement estimations. */
  private final Timer timer;

  /** The last time the timer was read, used to determine position changes. */
  private double lastTime;

  /** Heading of the robot. */
  private double angle;

  /** Create the swerve drive IMU simulation. */
  public SwerveGyroSimulation() {
    timer = new Timer();
    timer.start();
    lastTime = timer.get();
  }

  /**
   * Gets the estimated gyro {@link Rotation3d} of the robot.
   *
   * @return The heading as a {@link Rotation3d} angle
   */
  public Rotation3d getGyroRotation3d() {
    return new Rotation3d(0, 0, angle);
  }

  /**
   * Update the odometry of the simulated swerve drive
   *
   * @param kinematics {@link SwerveDriveKinematics} of the swerve drive.
   * @param states {@link SwerveModuleState} array of the module states.
   */
  public void updateOdometry(SwerveModuleState[] states) {
    if (RobotBase.isReal()) {
      return;
    }

    angle +=
        SwerveModuleConstants.kDriveKinematics.toChassisSpeeds(states).omegaRadiansPerSecond
            * (timer.get() - lastTime);
    lastTime = timer.get();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromRadians(angle);
  }

  /**
   * Set the heading of the robot.
   *
   * @param angle Angle of the robot in degrees.
   */
  public void setAngle(double angle) {
    if (RobotBase.isReal()) {
      alert.set(true);
      return;
    }

    this.angle = Math.toRadians(angle);
  }
}
