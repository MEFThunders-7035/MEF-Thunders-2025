package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.ArmPIDConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.utils.sim_utils.CANSparkMAXWrapped;

public class ArmSubsystem extends SubsystemBase implements AutoCloseable {
  private final CANSparkMAXWrapped armMotor;
  private final RelativeEncoder armEncoder;
  private final SparkClosedLoopController pidController;

  private ArmPosition desiredPosition = ArmPosition.IDLE;

  public enum ArmPosition {
    START(ArmConstants.kStartingAngle),
    GRAB(ArmConstants.kGrabAngle),
    SHOOT(ArmConstants.kShootAngle),
    IDLE(0);

    private double position;

    private ArmPosition(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }


  public ArmSubsystem() {
    armMotor = new CANSparkMAXWrapped(16, MotorType.kBrushless);
    setupSparkMax();
    armEncoder = armMotor.getEncoder();
    armEncoder.setPosition(ArmConstants.kStartingAngle);
    pidController = armMotor.getClosedLoopController();
  }

  @Override
  public void close() {
    armMotor.close();
  }

  private void setupSparkMax() {
    final SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig.encoder
    .positionConversionFactor(ArmConstants.kArmEncoderPositionFactor);

    armConfig
        .closedLoop
        .p(ArmPIDConstants.kP)
        .i(ArmPIDConstants.kI)
        .d(ArmPIDConstants.kD)
        .iMaxAccum(0.1);

    armConfig.inverted(true).idleMode(IdleMode.kBrake);

    armMotor.configure(armConfig);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder", armEncoder.getPosition());
  }

  public boolean atPosition() {
    return atPosition(desiredPosition);
  }

  public boolean atPosition(ArmPosition position) {
    return Math.abs(armEncoder.getPosition() - position.getPosition()) < ArmConstants.kArmTolerance;
  }

  public Command set(ArmPosition position) {
    desiredPosition = position;
    return run(() -> setPosition(position)).until(this::atPosition);
  }

  private void setPosition(ArmPosition position) {
    setPosition(position.getPosition());
  }

  private void setPosition(double position) {
    pidController.setReference(position, ControlType.kPosition);
  }
}
