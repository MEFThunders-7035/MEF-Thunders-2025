package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CageConstants;
import frc.utils.sim_utils.CANSparkMAXWrapped;

public class CageSubsystem extends SubsystemBase implements AutoCloseable {

  private final CANSparkMAXWrapped cageMotor;

  /** Creates a new CageSubsystem. */
  public CageSubsystem() {
    cageMotor = new CANSparkMAXWrapped(CageConstants.kCageIntakeMotorCanID, MotorType.kBrushed);

    configureSparkMAX();
    setDefaultCommand(runMotorCommand(CageConstants.kCageIdleSpeed));
  }

  private void configureSparkMAX() {
    var config = new SparkMaxConfig();

    config.smartCurrentLimit(40).inverted(false).idleMode(IdleMode.kBrake);

    cageMotor.configure(config);
  }

  @Override
  public void close() {
    cageMotor.close();
  }

  public Command openCage() {
    return runMotorCommand(CageConstants.kCageIntakeSpeed);
  }

  public Command closeCage() {

    return runMotorCommand(CageConstants.kCageCloseSpeed);
  }

  public double getCageIntakeSpeed() {
    return cageMotor.get();
  }

  public Command stop() {
    return runMotorCommand(0);
  }

  private Command runMotorCommand(double speed) {
    return this.runEnd(() -> setMotorSpeed(speed), this::stopMotor);
  }

  private void stopMotor() {
    cageMotor.stopMotor();
  }

  private void setMotorSpeed(double speed) {
    cageMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
