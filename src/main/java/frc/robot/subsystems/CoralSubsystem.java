package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;
import frc.utils.sim_utils.CANSparkMAXWrapped;
import frc.utils.sim_utils.ColorSensorV3Wrapped;

@Logged
public class CoralSubsystem extends SubsystemBase implements AutoCloseable {
  private final CANSparkMAXWrapped coralMotor =
      new CANSparkMAXWrapped(CoralIntakeConstants.kIntakeMotorCanID, MotorType.kBrushless);
  ColorSensorV3Wrapped colorSensor = new ColorSensorV3Wrapped(Port.kMXP);

  private double desiredSpeed;

  public CoralSubsystem() {
    configureSparkMAX();
  }

  private void configureSparkMAX() {
    var config = new SparkMaxConfig();

    config
        .smartCurrentLimit(50) // NEO stall current is 20A
        .inverted(false)
        .idleMode(IdleMode.kBrake);

    coralMotor.configure(config);
  }

  @Override
  public void close() {
    coralMotor.close();
    colorSensor.close();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Color Sensor Distance", colorSensor.getProximity());
    SmartDashboard.putNumber("Desired Speed", desiredSpeed);
  }

  public boolean hasCoral() {
    return colorSensor.getProximity() > CoralIntakeConstants.kProximityThreshold;
  }

  public Command takeCoral() {
    return runMotorCommand(CoralIntakeConstants.kIntakeSpeed).until(this::hasCoral);
  }

  public Command throwCoral() {
    return runMotorCommand(CoralIntakeConstants.kThrowSpeed);
  }

  public Command pushCoralOut() {
    return runMotorCommand(CoralIntakeConstants.kPushSpeed);
  }

  public Command moveBackwardForHelp() {
    return runMotorCommand(-CoralIntakeConstants.kIntakeSpeed);
  }

  public Command stop() {
    return runMotorCommand(0);
  }

  private Command runMotorCommand(double speed) {
    desiredSpeed = speed;
    return this.runEnd(() -> setMotorSpeed(speed), this::stopMotor).alongWith(Commands.print("Running coral motor at speed: "+ speed));
  }

  private void stopMotor() {
    coralMotor.set(0);
  }

  private void setMotorSpeed(double speed) {
    coralMotor.set(speed);
  }
}
