package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.DrivePIDController;
import frc.robot.Constants.AutoConstants.RotationPIDController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.MotorConstants;
import frc.robot.Constants.DriveConstants.SwerveModuleConstants;
import frc.robot.simulationSystems.SwerveGyroSimulation;
import frc.utils.ExtraFunctions;
import frc.utils.SwerveUtils;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  private final AHRS navX = new AHRS(NavXComType.kMXP_SPI);
  private Field2d field = new Field2d();

  private Rotation2d fieldOrientationRotateBy = new Rotation2d();
  private boolean forceRobotOriented = false;

  private final MAXSwerveModule frontLeft =
      new MAXSwerveModule(
          MotorConstants.kFrontLeftDrivingCanID,
          MotorConstants.kFrontLeftTurningCanID,
          SwerveModuleConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule frontRight =
      new MAXSwerveModule(
          MotorConstants.kFrontRightDrivingCanID,
          MotorConstants.kFrontRightTurningCanID,
          SwerveModuleConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule rearLeft =
      new MAXSwerveModule(
          MotorConstants.kRearLeftDrivingCanID,
          MotorConstants.kRearLeftTurningCanID,
          SwerveModuleConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule rearRight =
      new MAXSwerveModule(
          MotorConstants.kRearRightDrivingCanID,
          MotorConstants.kRearRightTurningCanID,
          SwerveModuleConstants.kBackRightChassisAngularOffset);

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double prevTime = WPIUtilJNI.now() * 1e-6;

  SwerveGyroSimulation gyroSim =
      new SwerveGyroSimulation(); // required for getRotation2d() in simulation

  SwerveDrivePoseEstimator swerveOdometry =
      new SwerveDrivePoseEstimator(
          SwerveModuleConstants.kDriveKinematics,
          getRotation2d(),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
          },
          new Pose2d());

  StructArrayPublisher<SwerveModuleState> publisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Swerve", SwerveModuleState.struct)
          .publish();

  Vision visionSystem;

  public DriveSubsystem() {
    visionSystem = new Vision(this::getPose);

    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            this.driveRobotRelative(
                speeds), // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(
                DrivePIDController.kP, DrivePIDController.kD), // Translation PID constants
            new PIDConstants(
                RotationPIDController.kP, RotationPIDController.kD)), // Rotation PID constants
        // Max module speed, in m/s)
        AutoConstants.kRobotConfig.get(),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
    SmartDashboard.putNumber("Move By", 0);
  }

  @Override
  public void periodic() {
    var pose =
        swerveOdometry.update(
            getRotation2d(),
            new SwerveModulePosition[] {
              frontLeft.getPosition(),
              frontRight.getPosition(),
              rearLeft.getPosition(),
              rearRight.getPosition()
            });
    updatePoseWithVision();
    field.setRobotPose(pose);
    gyroSim.updateOdometry(getModuleDesiredStates());
    SmartDashboard.putData(field);
    SmartDashboard.putNumber("Rotation", getRotation2d().getDegrees());
    SmartDashboard.putNumber("Rotation-Radians", getRotation2d().getRadians());
    SmartDashboard.putBoolean("Force Robot Oriented", forceRobotOriented);

    publisher.set(
        new SwerveModuleState[] {
          frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState(),
        });

    CommandScheduler.getInstance().printWatchdogEpochs();
  }

  @Override
  public void simulationPeriodic() {
    visionSystem.simulationPeriodic();
  }

  @Override
  public void close() {
    frontLeft.close();
    frontRight.close();
    rearLeft.close();
    rearRight.close();

    field.close();
    publisher.close();
    visionSystem.close();

    navX.close();
  }

  private void updatePoseWithVision() {
    var poseOpt = visionSystem.getEstimatedGlobalPose();
    SmartDashboard.putBoolean("AprilTag Seen", poseOpt.isPresent());
    // only allow multitag values
    SmartDashboard.putNumber("Distance To Shooter", getDistanceToShooter());

    SmartDashboard.putNumber(
        "Rotation Difference to Shooter", getRotationDifferenceToShooter().getDegrees());

    SmartDashboard.putNumber(
        "Arm Angle Required", ExtraFunctions.getAngleFromDistance(getDistanceToShooter()));

    if (poseOpt.isEmpty()) return;

    var pose = poseOpt.get();
    if (pose.targetsUsed.size() < 2) return; // is multiTag

    // Do not use the rotation from the vision system in any situation as the data
    // we receive is not reliable.
    // navX rotation is A LOT MORE reliable so we will use that instead.
    Pose2d receivedPose = pose.estimatedPose.toPose2d();
    Pose2d poseToUse = new Pose2d(receivedPose.getTranslation(), getRotation2d());
    swerveOdometry.addVisionMeasurement(poseToUse, pose.timestampSeconds);
  }

  private Optional<Pose3d> getTagPose(int id) {
    var tag = Vision.fieldLayout.getTagPose(id);

    if (tag.isEmpty()) {
      DriverStation.reportError("Field Layout Couldn't be loaded", false);
      return Optional.empty();
    }

    return tag;
  }

  /**
   * Gets the distance to the shooter.
   *
   * @exception DriverStation.reportError if the field layout couldn't be loaded. and returns 0.
   * @return returns the distance to the shooter in meters. will return 0 if the field layout
   *     couldn't be loaded.
   */
  public double getDistanceToShooter() {
    var tag = getTagPose(ExtraFunctions.getShooterAprilTagID());
    if (tag.isEmpty()) return 0;

    return tag.get().getTranslation().toTranslation2d().getDistance(getPose().getTranslation())
        + SmartDashboard.getNumber("Move By", 0);
  }

  public Rotation2d getRotationDifferenceToShooter() {
    var tag = getTagPose(ExtraFunctions.getShooterAprilTagID());

    if (tag.isEmpty()) return new Rotation2d();

    Rotation2d robotFontsRotationDifferenceToShooter =
        tag.get()
            .getTranslation() // Get the translation of the tag
            .toTranslation2d()
            .minus(getPose().getTranslation()) // Subtract the robot's translation
            .getAngle()
            .minus(getRotation2d()); // Subtract the robot's rotation

    return robotFontsRotationDifferenceToShooter
        .rotateBy(Rotation2d.fromDegrees(180)) // Rotate by 180 so the back is 0
        .times(-1); // Invert the angle, so the back is a positive angle that can be inverted.
  }

  public Pose2d getPose() {
    return swerveOdometry.getEstimatedPosition();
  }

  public void setForceRobotOriented(boolean forceRobotOriented) {
    this.forceRobotOriented = forceRobotOriented;
  }

  public void toggleForceRobotOriented() {
    forceRobotOriented = !forceRobotOriented;
  }

  public boolean isForceRobotOriented() {
    return forceRobotOriented;
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(
        getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
        },
        pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return SwerveModuleConstants.kDriveKinematics.toChassisSpeeds(
        frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState());
  }

  /* ACTUALLY DRIVE RELATED CODE */

  /**
   * Method to drive the robot using the ChassisSpeeds object. Should most probably only be used in
   * the path following system. and no where else. If not, you are probably doing something wrong
   * and should use {@link #drive(double, double, double)} instead.
   *
   * @param speeds The ChassisSpeeds object to drive the robot with.
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    var swerveModuleStates = SwerveModuleConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public Command setX() {
    return this.runOnce(this::setModulesToXFormation);
  }

  public Command goToPose(Pose2d pose) {
    return AutoBuilder.pathfindToPose(pose, AutoConstants.kPathConstraints);
  }

  /**
   * Method to drive the robot in a field relative way
   *
   * @param xSpeed Speed of the robot in the x direction (forward). + is forward
   * @param ySpeed Speed of the robot in the y direction (sideways). + is left of robot see wpilib
   *     coordinate system for more info
   *     (https://docs.wpilib.org/tr/stable/docs/software/basic-programming/coordinate-system.html)
   * @param rot the rotation of the robot. + is counterclockwise again see wpilib coordinate system
   */
  public Command drive(
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed,
      DoubleSupplier rot,
      BooleanSupplier fieldRelative,
      BooleanSupplier rateLimit) {
    return this.runEnd(
        () ->
            driveRobot(
                xSpeed.getAsDouble(),
                ySpeed.getAsDouble(),
                rot.getAsDouble(),
                fieldRelative.getAsBoolean(),
                rateLimit.getAsBoolean()),
        this::stopRobot);
  }

  public Command drive(
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed,
      DoubleSupplier rot,
      boolean fieldRelative,
      boolean rateLimit) {
    return drive(xSpeed, ySpeed, rot, () -> fieldRelative, () -> rateLimit);
  }

  public Command drive(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot) {
    return drive(xSpeed, ySpeed, rot, true, true);
  }

  public Command drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    return drive(() -> xSpeed, () -> ySpeed, () -> rot, fieldRelative, rateLimit);
  }

  public Command drive(double xSpeed, double ySpeed, double rot) {
    return drive(xSpeed, ySpeed, rot, true, true);
  }

  public Command stop() {
    return this.runOnce(this::stop);
  }

  public Command resetFieldOrientation() {
    return this.runOnce(this::zeroFieldOrientation)
        .alongWith(new PrintCommand("Zeroing Field Orientation"))
        .ignoringDisable(true);
  }

  private void stopRobot() {
    driveRobot(0, 0, 0, false, false);
  }

  /**
   * Method to drive the robot using joystick info. Taken straight from REV MAXSwerve Template.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   */
  private void driveRobot(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (forceRobotOriented) {
      fieldRelative = false;
    }

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively
        // instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (currentTranslationMag
            > 1e-4) { // some small number to avoid floating-point errors with equality
          // checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        } else {
          currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      } else {
        currentTranslationDir =
            SwerveUtils.StepTowardsCircular(
                currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      prevTime = currentTime;

      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
      currentRotation = rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates =
        SwerveModuleConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered, getFieldOrientedRotation2d())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    setModuleStates(swerveModuleStates);
  }

  private void setModulesToXFormation() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  private void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleDesiredStates() {
    return new SwerveModuleState[] {
      frontLeft.getDesiredState(),
      frontRight.getDesiredState(),
      rearLeft.getDesiredState(),
      rearRight.getDesiredState()
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  /**
   * Zeroes the heading of the robot.
   *
   * @apiNote THIS SHOULD NOT BE USED IN A MATCH, USE ZERO FIELD ORIENTATION INSTEAD ONLY USE THIS
   *     IN AN EMERGENCY AS IT WILL FUCK UP THE VISION AND OTHER ODOMETRY SYSTEMS
   */
  public void zeroHeading() {
    navX.reset();
  }

  /**
   * Zeros the heading for the field orientation system, so that it is easier to use looking trough
   * a different orientation.
   *
   * @see {@link zeroHeading} for actually resetting the heading on the hardware level
   */
  public void zeroFieldOrientation() {
    fieldOrientationRotateBy = getRotation2d().unaryMinus().rotateBy(Rotation2d.fromDegrees(0));
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return navX.getAngle();
  }

  public Rotation2d getRotation2d() {
    if (RobotBase.isSimulation() && navX.getAngle() == 0) {
      return gyroSim.getRotation2d();
    }
    return navX.getRotation2d();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navX.getRate();
  }

  private Rotation2d getFieldOrientedRotation2d() {
    return getRotation2d().rotateBy(fieldOrientationRotateBy);
  }

  // ONLY FOR SIMULATION

  /**
   * Set the simulated gyro angle.
   *
   * <p>THIS IS FOR SIMULATION ONLY WILL NOT WORK OTHERWISE. DO NOT USE IN A REAL ROBOT.
   *
   * @param angle the angle in degrees
   */
  public void setSimulatedGyroAngle(double angle) {
    gyroSim.setAngle(angle);
  }
}
