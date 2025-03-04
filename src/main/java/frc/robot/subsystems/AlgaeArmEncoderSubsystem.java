package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.AlgaeArmConstants.AlgaeArmPIDConstants;
import frc.utils.sim_utils.CANSparkMAXWrapped;

public class AlgaeArmEncoderSubsystem extends SubsystemBase implements AutoCloseable {

  private final CANSparkMAXWrapped algaeArmMotor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pidController;
  private double desiredPosition = 0;

  public AlgaeArmEncoderSubsystem() {
    // Algae Arm motor initialization
    algaeArmMotor = new CANSparkMAXWrapped(10, MotorType.kBrushed);

    setupSparkMax(); // Ensure the motor is properly configured before initializing the encoder

    encoder = algaeArmMotor.getEncoder();
    pidController = algaeArmMotor.getClosedLoopController();
    resetEncoder();
  }

  SparkMaxConfig algaeArmMotorConfig2 = new SparkMaxConfig();

  private void setupSparkMax() {
    SparkMaxConfig algaeArmMotorConfig = new SparkMaxConfig();
    algaeArmMotorConfig
        .encoder
        .countsPerRevolution(4096) // Encoder CPR for Algae Arm motor
        .positionConversionFactor(1.0);

    algaeArmMotorConfig
        .closedLoop
        .p(AlgaeArmPIDConstants.kP)
        .i(AlgaeArmPIDConstants.kI)
        .d(AlgaeArmPIDConstants.kD)
        .iMaxAccum(0.1)
        .outputRange(-0.2, 0.2);

    algaeArmMotorConfig
        .inverted(false)
        .idleMode(SparkMaxConfig.IdleMode.kBrake)
        .smartCurrentLimit(40);

    algaeArmMotor.configure(algaeArmMotorConfig);
  }

  private void setupSparkMax2() {
    SparkMaxConfig algaeArmMotorConfig2 = new SparkMaxConfig();
    algaeArmMotorConfig2
        .encoder
        .countsPerRevolution(4096) // Encoder CPR for Algae Arm motor
        .positionConversionFactor(1.0);

    algaeArmMotorConfig2
        .closedLoop
        .p(AlgaeArmPIDConstants.kP)
        .i(AlgaeArmPIDConstants.kI)
        .d(AlgaeArmPIDConstants.kD)
        .iMaxAccum(0.1)
        .outputRange(-0.2, 0.7);

    algaeArmMotorConfig2
        .inverted(true)
        .idleMode(SparkMaxConfig.IdleMode.kBrake)
        .smartCurrentLimit(40);

    algaeArmMotor.configure(algaeArmMotorConfig2);
  }

  @Override
  public void close() {
    algaeArmMotor.close();
  }

  @Override
  public void periodic() {
    // Update the dashboard with the current encoder position

    SmartDashboard.putNumber("Algae Arm Encoder Position", encoder.getPosition());

    if (encoder.getPosition() > -0.5 && encoder.getPosition() < 0.6) {
      encoder.setPosition(0.6);
    }
  }

  public boolean isAlgaeArmAtPosition(double position) {
    position = MathUtil.clamp(position, 0, 1.0); // Assuming the range is between 0 and 1

    return Math.abs(encoder.getPosition() - position) < AlgaeArmPIDConstants.kAllowedError;
  }

  public double getAlgaeArmPosition() {
    return encoder.getPosition();
  }

  public boolean isAlgaeArmAtPosition() {
    return isAlgaeArmAtPosition(desiredPosition);
  }

  public double getDesiredPosition() {
    return desiredPosition;
  }

  public void resetEncoder() {
    resetEncoder(0.55);
  }

  public void resetEncoder(double position) {
    encoder.setPosition(position);
  }

  public Command moveArmToPosition(double position) {
    return runEnd(() -> setAlgaeArmPosition(position), this::stopAlgaeArm)
        .until(this::isAlgaeArmAtPosition);
  }

  public Command stop() {
    return runOnce(this::stopAlgaeArm);
  }

  private void stopAlgaeArm() {
    algaeArmMotor.stopMotor();
  }

  public void setAlgaeArmPosition(double position) {
    pidController.setReference(position, ControlType.kPosition);
  }

  public Command setArmDirection(boolean reverse) {
    // Eğer motor 'inverted' değilse, ters yap
    if (algaeArmMotor.configAccessor.getInverted() == false) {
      setupSparkMax2(); // Motoru ters yönlendirecek konfigürasyonu uygula
    } else {
      setupSparkMax(); // Motoru eski (normal) yönüne alacak konfigürasyonu uygula
    }

    return set(0); // Pozisyonu sıfırlama işlemi
  }

  public Command runMotorTest(double speed) {
    algaeArmMotor.set(speed);
    return stop(); // You can stop the motor after running the test
  }

  public Command set(double position) {
    return runEnd(() -> setAlgaeArmPosition(position), this::stopAlgaeArm)
        .until(this::isAlgaeArmAtPosition);
  }

  public Command setArmToAmp() {
    return set(AlgaeArmConstants.AlgaeArmPIDConstants.AMP_POSITION);
  }
}
