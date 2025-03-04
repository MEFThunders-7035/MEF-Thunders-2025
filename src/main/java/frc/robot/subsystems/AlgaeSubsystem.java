package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.utils.sim_utils.CANSparkMAXWrapped;

public class AlgaeSubsystem extends SubsystemBase implements AutoCloseable {

  private final CANSparkMAXWrapped algaeMotor;

  public AlgaeSubsystem() {
    algaeMotor =
        new CANSparkMAXWrapped(AlgaeIntakeConstants.kAlgaeIntakeMotorCanID, MotorType.kBrushless);
    configureSparkMAX();
    setDefaultCommand(runMotorAlgaeCommand(AlgaeIntakeConstants.kAlgaeIdleSpeed));
  }

  private void configureSparkMAX() {
    var config = new SparkMaxConfig();

    config
        .smartCurrentLimit(20) // NEO 550 stall current is 20A
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    algaeMotor.configure(config);
  }

  @Override
  public void close() {
    algaeMotor.close();
  }

  @Override
  public void periodic() {}

  public Command takeAlgae() {
    return runMotorAlgaeCommand(AlgaeIntakeConstants.kAlgaeIntakeSpeed);
  }

  public Command throwAlgae() {
    return runMotorAlgaeCommand(AlgaeIntakeConstants.kAlgaeThrowSpeed);
  }

  public Command stop() {
    return runMotorAlgaeCommand(0);
  }

  public Command runMotorAlgaeCommand(double speed) {
    return this.runEnd(() -> setMotorSpeed(speed), this::stopMotor);
  }

  private void stopMotor() {
    algaeMotor.stopMotor();
  }

  private void setMotorSpeed(double speed) {
    algaeMotor.set(speed);
  }
}
