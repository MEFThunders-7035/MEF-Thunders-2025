package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;
import frc.utils.sim_utils.CANSparkMAXWrapped;

public class CoralSubsystem extends SubsystemBase implements AutoCloseable {
  private final CANSparkMAXWrapped coralMotor;

  public CoralSubsystem() {
    coralMotor =
        new CANSparkMAXWrapped(CoralIntakeConstants.kIntakeMotorCanID, MotorType.kBrushless);

    setDefaultCommand(runMotorCommand(CoralIntakeConstants.kIdleSpeed));
  }

  @Override
  public void close() {
    coralMotor.close();
  }

  public boolean hasCoral() {
    // Will be implemented
    return true;
  }

  public Command takeCoral() {
    return runMotorCommand(CoralIntakeConstants.kIntakeSpeed);
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
