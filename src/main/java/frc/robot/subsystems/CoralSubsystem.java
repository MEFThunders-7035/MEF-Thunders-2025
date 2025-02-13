package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;
import frc.utils.sim_utils.CANSparkMAXWrapped;

public class CoralSubsystem extends SubsystemBase implements AutoCloseable {
  private final CANSparkMAXWrapped coralMotor;
  private final Ultrasonic ultrasonicSensor;

  public CoralSubsystem() {
    coralMotor =
        new CANSparkMAXWrapped(CoralIntakeConstants.kIntakeMotorCanID, MotorType.kBrushless);

    ultrasonicSensor =
        new Ultrasonic(
            CoralIntakeConstants.kUltrasonicPingChannel,
            CoralIntakeConstants.kUltrasonicEchoChannel);

    configureSparkMAX();
    setDefaultCommand(runMotorCommand(CoralIntakeConstants.kIdleSpeed));
  }

  private void configureSparkMAX() {
    var config = new SparkMaxConfig();

    config
        .smartCurrentLimit(50) // NEO stall current is 50A
        .inverted(false)
        .idleMode(IdleMode.kBrake);

    coralMotor.configure(config);
  }

  @Override
  public void close() {
    coralMotor.close();
    ultrasonicSensor.close();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ultrasonic Distance", ultrasonicSensor.getRangeMM());
  }

  public boolean hasCoral() {
    // Will be implemented
    return ultrasonicSensor.getRangeMM() != 0 // Something is wrong
        && ultrasonicSensor.getRangeMM() < CoralIntakeConstants.kUltrasonicThreshold;
  }

  public Command takeCoral() {

    return runMotorCommand(CoralIntakeConstants.kIntakeSpeed).until(this::hasCoral);
  }

  public Command throwCoral() {
    return runMotorCommand(CoralIntakeConstants.kThrowSpeed);
  }

  public Command stop() {
    return runMotorCommand(0);
  }

  private Command runMotorCommand(double speed) {
    return this.runEnd(() -> setMotorSpeed(speed), this::stopMotor);
  }

  private void stopMotor() {
    coralMotor.stopMotor();
  }

  private void setMotorSpeed(double speed) {
    coralMotor.set(speed);
  }
}
