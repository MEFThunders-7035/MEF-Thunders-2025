package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.ArmPIDConstants;
import frc.utils.ExtraFunctions;
import frc.utils.sim_utils.CANSparkMAXWrapped;
import java.util.function.DoubleSupplier;

public class ArmSubsystem extends SubsystemBase implements AutoCloseable {
  // We only have RelativeEncoder for now. Its better than nothing.
  private final CANSparkMAXWrapped arm;
  private final CANSparkMAXWrapped armFollower;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pidController;
  private final ArmFeedforward feedforward;

  private static final double kArmParallelDifference = 0.00339;

  private double desiredPosition = 0;

  public enum ArmState {
    AMP,
    BOTTOM,
    IDLE
  }

  public ArmSubsystem() {
    arm = new CANSparkMAXWrapped(IntakeConstants.kArmMotorCanID, MotorType.kBrushed);
    armFollower =
        new CANSparkMAXWrapped(IntakeConstants.kArmFollowerMotorCanID, MotorType.kBrushed);

    setupSparkMax();

    encoder = arm.getEncoder();
    pidController = arm.getClosedLoopController();
    feedforward = new ArmFeedforward(ArmPIDConstants.kS, ArmPIDConstants.kG, ArmPIDConstants.kV);

    SmartDashboard.putNumber("Arm Pos", 0);
  }

  /**
   * Sets up the Spark Max for the arm. This includes setting the PID constants, setting the
   * conversion factors for the encoder, setting the idle mode, and setting the smart current limit.
   * This method is here incase we want to switch to a different motor controller.
   */
  private void setupSparkMax() {
    SparkMaxConfig armFollowerConfig = new SparkMaxConfig();
    armFollowerConfig.follow(arm, true).smartCurrentLimit(IntakeConstants.kSmartCurrentLimit);
    armFollower.configure(armFollowerConfig);

    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig
        .encoder
        .countsPerRevolution(IntakeConstants.kArmEncoderCPR)
        .positionConversionFactor(IntakeConstants.kArmEncoderPositionFactor);

    armConfig
        .closedLoop
        .p(IntakeConstants.ArmPIDConstants.kP)
        .i(IntakeConstants.ArmPIDConstants.kI)
        .d(IntakeConstants.ArmPIDConstants.kD)
        .iMaxAccum(0.1)
        .outputRange(-0.2, 0.7); // I don't think we need these as Constants in a file.

    armConfig
        .inverted(false)
        .idleMode(IntakeConstants.kArmMotorIdleMode)
        .smartCurrentLimit(IntakeConstants.kSmartCurrentLimit);

    arm.configure(armConfig);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder Position", encoder.getPosition());
  }

  @Override
  public void close() {
    arm.close();
    armFollower.close();
  }

  /**
   * Checks if the arm is at a given position within a certain error. see {@link
   * ArmPIDConstants#kAllowedError} for the allowed error.
   *
   * @param position The position to check if the arm is at.
   * @return true if the arm is at the desired position.
   */
  public boolean isArmAtPosition(double position) {
    position = MathUtil.clamp(position, 0, 0.5);
    return Math.abs(encoder.getPosition() - position) < ArmPIDConstants.kAllowedError;
  }

  /**
   * Checks if the arm is at the last set desired position within a certain error. see {@link
   * ArmPIDConstants#kAllowedError} for the allowed error.
   *
   * @return true if the arm is at the desired position.
   */
  public boolean isArmAtPosition() {
    return isArmAtPosition(desiredPosition);
  }

  public double getDesiredPosition() {
    return desiredPosition;
  }

  /**
   * Sets the current position of the arm as the starting position. This is useful for when the arm
   * is at rest like when starting the robot, and we wish to calibrate the arm encoder.
   *
   * <p>Warning: Use with caution and only call this while the arm is at rest! or this will cause
   * all sorts of problems.
   */
  public void resetEncoder() {
    resetEncoder(0);
  }

  /**
   * Sets the current position of the arm as the starting position. This is useful if there is a
   * limit switch at a known position and we wish to calibrate the arm encoder to that position.
   *
   * @param position the position of the arm in rotations. (from 0 to 1 for half a rotation ie 180
   *     deg) and since the arm can only go until 90 deg, we will only use 0 to 0.5, do not set as
   *     anything else.
   */
  public void resetEncoder(double position) {
    encoder.setPosition(position);
  }

  public Command stop() {
    return this.runOnce(this::stopArm);
  }

  public Command set(ArmState state) {
    switch (state) {
      case AMP:
        return setArmToAmp();
      case BOTTOM:
        return setArmToBottom();
      case IDLE:
        return stop();
      default:
        return stop();
    }
  }

  public Command setArmToAmp() {
    return set(ArmConstants.AMP_POSITION);
  }

  public Command setArmToBottom() {
    return set(0.0);
  }

  /**
   * Warning, command will not end unless interrupted since you are probably using this command
   * while moving around
   *
   * @return The command object that sets the arm to the shooter.
   */
  public Command setArmToShooter(DoubleSupplier distanceToShooter) {
    return this.runEnd(
        () -> ExtraFunctions.getAngleFromDistance(distanceToShooter.getAsDouble()), this::stopArm);
  }

  public Command set(double position) {
    return this.runEnd(() -> setArmToPosition(position), this::stopArm)
        .until(this::isArmAtPosition);
  }

  private void stopArm() {
    arm.stopMotor();
  }

  /**
   * Calculates the feedforward for the arm. Since the arm is slightly down when its at "0" we
   * compensate for that in the calculation by subtracting {@link #kArmParallelDifference} from the
   * position.
   *
   * @param position The position to calculate the feedforward for.
   * @return The calculated feedforward.
   */
  private double calculateFeedForward(double position) {
    var calculation =
        feedforward.calculate(
            (position - kArmParallelDifference) * Math.PI,
            Math.abs(encoder.getPosition() - position));

    SmartDashboard.putNumber("FeedForward Calculation", calculation);
    SmartDashboard.putNumber("Setpoint", position);

    return calculation;
  }

  /**
   * Sets the arm to a given position.
   *
   * @param position The rotation the arm should be at. (from 0 to 1)
   * @see #setArmToPosition(int)
   */
  private void setArmToPosition(double position) {
    position = MathUtil.clamp(position, 0, 0.5);
    desiredPosition = position;

    double feedForwardCalculation = calculateFeedForward(position);
    pidController.setReference(
        position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedForwardCalculation,
        ArbFFUnits.kVoltage);
  }
}
