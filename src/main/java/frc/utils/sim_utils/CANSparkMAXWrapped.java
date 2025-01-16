package frc.utils.sim_utils;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

public class CANSparkMAXWrapped extends SparkMax {
  private Alert configFailedAlert =
      new Alert(
          "SparkMAX with ID: " + this.getDeviceId() + "Configuration Failed", AlertType.kError);

  public CANSparkMAXWrapped(int deviceID, MotorType type) {
    super(deviceID, type);
    if (RobotBase.isSimulation()) {
      SparkMAXSimAddon.addSparkMAX(this);
    }
  }

  @Override
  public void close() {
    if (RobotBase.isSimulation()) {
      SparkMAXSimAddon.removeSparkMAX(this);
    }
    super.close();
  }

  /**
   * Set the configuration for the SPARK.
   *
   * <p>this method will reset safe writable parameters to their default values before setting the
   * given configuration. The following parameters will not be reset by this action: CAN ID, Motor
   * Type, Idle Mode, PWM Input Deadband, and Duty Cycle Offset.
   *
   * <p>This method will save all parameters to the SPARK's non-volatile memory after setting the
   * given configuration. This will allow parameters to persist across power cycles.
   *
   * @see #configure(SparkMaxConfig, ResetMode, PersistMode) To change the reset and persistence
   * @param config The desired SPARK configuration
   * @return {@link REVLibError#kOk} if successful
   */
  public REVLibError configure(SparkMaxConfig config) {
    var isConfigurationSuccessful =
        this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (isConfigurationSuccessful != REVLibError.kOk) {
      configFailedAlert.set(true);
    }

    return isConfigurationSuccessful;
  }

  public boolean isThisClosed() {
    return isClosed.get();
  }
}
