package frc.utils.sim_utils;

import com.revrobotics.CANSparkMax;
import java.util.HashMap;
import java.util.Map;

public class SparkMAXSimAddon {
  private static Map<Integer, CANSparkMax> sparkMaxes = new HashMap<>();

  private static void throwIfSparkMAXExists(int deviceID) {
    if (sparkMaxes.containsKey(deviceID)) {
      throw new IllegalArgumentException("SparkMAX with deviceID " + deviceID + " already exists");
    }
  }

  private static void throwIfSparkMAXExists(CANSparkMax sparkMAX) {
    throwIfSparkMAXExists(sparkMAX.getDeviceId());
  }

  private static void throwIfSparkMAXDoesNotExist(int deviceID) {
    if (!sparkMaxes.containsKey(deviceID)) {
      throw new IllegalArgumentException("SparkMAX with deviceID " + deviceID + " does not exist");
    }
  }

  private static void throwIfSparkMAXDoesNotExist(CANSparkMax sparkMAX) {
    throwIfSparkMAXDoesNotExist(sparkMAX.getDeviceId());
  }

  public static void addSparkMAX(CANSparkMax sparkMAX) {
    throwIfSparkMAXExists(sparkMAX);
    sparkMaxes.put(sparkMAX.getDeviceId(), sparkMAX);
  }

  public static void removeSparkMAX(CANSparkMax sparkMAX) {
    throwIfSparkMAXDoesNotExist(sparkMAX);
    sparkMaxes.remove(sparkMAX.getDeviceId());
  }

  public static CANSparkMax getSparkMAX(int deviceID) {
    throwIfSparkMAXDoesNotExist(deviceID);
    return sparkMaxes.get(deviceID);
  }

  public static boolean doesSparkMAXExist(int deviceID) {
    return sparkMaxes.containsKey(deviceID);
  }

  /** Closes all SparkMAXes and clears the data. */
  public static void resetData() {
    for (CANSparkMax sparkMAX : sparkMaxes.values()) {
      sparkMAX.close();
    }
    sparkMaxes.clear();
  }
}
