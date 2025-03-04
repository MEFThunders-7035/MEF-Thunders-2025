package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.ElevatorFeedForwardConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.utils.sim_utils.CANSparkMAXWrapped;

public class ElevatorSubsystem extends SubsystemBase implements AutoCloseable {
  private final CANSparkMAXWrapped elevatorMotor =
      new CANSparkMAXWrapped(ElevatorConstants.kElevatorMotorCanID, MotorType.kBrushed);
  private final CANSparkMAXWrapped elevatorMotorFollower =
      new CANSparkMAXWrapped(ElevatorConstants.kElevatorMotorFollowerCanID, MotorType.kBrushed);

  ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(kS, kG, kV);

  private final RelativeEncoder elevatorEncoder;
  private final SparkClosedLoopController pidController;

  private ElevatorPosition desiredPosition = ElevatorPosition.IDLE;

  public enum ElevatorPosition {
    L1(ElevatorConstants.kElevatorL1Height),
    L2(ElevatorConstants.kElevatorL2Height),
    L3(ElevatorConstants.kElevatorL3Height),
    L4(ElevatorConstants.kElevatorL4Height),
    IDLE(0);

    private double position;

    private ElevatorPosition(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }

  public ElevatorSubsystem() {
    setupSparkMAX();

    elevatorEncoder = elevatorMotor.getEncoder();
    pidController = elevatorMotor.getClosedLoopController();
  }

  @Override
  public void close() {
    elevatorMotor.close();
    elevatorMotorFollower.close();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder Value", elevatorEncoder.getPosition());
  }

  public boolean atPosition() {
    return Math.abs(elevatorEncoder.getPosition() - desiredPosition.getPosition())
        < ElevatorConstants.kElevatorTolerance;
  }

  private void setupSparkMAX() {
    final SparkMaxConfig followerConfig = new SparkMaxConfig();

    followerConfig.follow(elevatorMotor, true).idleMode(IdleMode.kBrake);

    final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    elevatorConfig
        .encoder
        .countsPerRevolution(ElevatorConstants.kElevatorEncoderCPR)
        .positionConversionFactor(ElevatorConstants.kElevatorEncoderPositionFactor);

    elevatorConfig
        .closedLoop
        .p(ElevatorConstants.ElevatorPIDConstants.kP)
        .i(ElevatorConstants.ElevatorPIDConstants.kI)
        .d(ElevatorConstants.ElevatorPIDConstants.kD)
        .iMaxAccum(0.1);

    elevatorConfig.inverted(false).idleMode(IdleMode.kBrake);

    elevatorMotor.configure(elevatorConfig);
    elevatorMotorFollower.configure(followerConfig);
  }

  public Command set(ElevatorPosition position) {
    desiredPosition = position;
    return run(() -> setPosition(position)).until(this::atPosition);
  }

  private void setPosition(ElevatorPosition position) {
    setPosition(position.getPosition());
  }

  private void setPosition(double position) {
    pidController.setReference(
        position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        elevatorFeedforward.calculate(position),
        ArbFFUnits.kVoltage);
  }
}
