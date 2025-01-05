package frc.utils.sim_utils;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.RobotBase;

public class CANSparkMAXWrapped extends CANSparkMax {
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

  public boolean isThisClosed() {
    return isClosed.get();
  }
}
