package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.simulationSystems.SwerveModuleSim;
import frc.utils.sim_utils.CANSparkMAXWrapped;

public class MAXSwerveModule implements AutoCloseable {
  private final CANSparkMAXWrapped m_drivingSparkMax;
  private final CANSparkMAXWrapped m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /* SIM RELATED OBJECTS */

  private final SwerveModuleSim driveSim = new SwerveModuleSim();

  /* END SIM RELATED OBJECTS */

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new CANSparkMAXWrapped(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMAXWrapped(turningCANId, MotorType.kBrushless);

    var drivingSparkMaxConfig = new SparkMaxConfig();
    var turningSparkMaxConfig = new SparkMaxConfig();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
    m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();

    drivingSparkMaxConfig
        .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
        .idleMode(ModuleConstants.kDrivingMotorIdleMode);

    turningSparkMaxConfig
        .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)
        .idleMode(ModuleConstants.kTurningMotorIdleMode);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    drivingSparkMaxConfig
        .encoder
        .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
        .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turningSparkMaxConfig
        .absoluteEncoder
        .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
        .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)
        .inverted(ModuleConstants.kTurningEncoderInverted);

    drivingSparkMaxConfig
        .closedLoop
        .p(ModuleConstants.kDrivingP)
        .i(ModuleConstants.kDrivingI)
        .d(ModuleConstants.kDrivingD)
        .velocityFF(ModuleConstants.kDrivingFF)
        .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    turningSparkMaxConfig
        .closedLoop
        .p(ModuleConstants.kTurningP)
        .i(ModuleConstants.kTurningI)
        .d(ModuleConstants.kTurningD)
        .velocityFF(ModuleConstants.kTurningFF)
        .outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput)
        .positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.configure(drivingSparkMaxConfig);
    m_turningSparkMax.configure(turningSparkMaxConfig);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  @Override
  public void close() {
    m_drivingSparkMax.close();
    m_turningSparkMax.close();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    if (RobotBase.isSimulation()) {
      return driveSim.getState();
    }
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    if (RobotBase.isSimulation()) {
      return driveSim.getPosition();
    }
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    m_desiredState = desiredState;

    if (RobotBase.isSimulation()) {
      driveSim.updateStateAndPosition(desiredState);
    }

    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
        desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    if (RobotBase.isSimulation()) {
      // For some reason, REVLib breaks `stepTime` in simulation,
      // so we do not call the SPARKS MAX PID controllers in simulation.
      // at least not until REV fixes the damn thing...
      return; // Don't set the SPARKS MAX in simulation.
    }

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(
        correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningPIDController.setReference(
        correctedDesiredState.angle.getRadians(), ControlType.kPosition);
  }

  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public Pose2d getRealWorldPose(Pose2d robotPose) {
    Pose2d moduleOffsetPose = getPoseDiffToRobot();

    return new Pose2d(
        robotPose.getX() + moduleOffsetPose.getX(),
        robotPose.getY() + moduleOffsetPose.getY(),
        getPosition().angle);
  }

  private Pose2d getPoseDiffToRobot() {
    var moduleRotationOffset = Rotation2d.fromRadians(m_chassisAngularOffset);
    var moduleOffsetLength = DriveConstants.kWheelBase / 2.0;

    return new Pose2d(
        moduleRotationOffset.getCos() * moduleOffsetLength,
        moduleRotationOffset.getSin() * moduleOffsetLength,
        moduleRotationOffset);
  }
}
