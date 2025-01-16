package frc.utils.sim_utils;

import com.revrobotics.spark.SparkMax;
import java.util.HashMap;
import java.util.Map;

public class SparkMAXSimAddon {
  private static Map<Integer, SparkMax> sparkMaxes = new HashMap<>(0);

  private static void throwIfSparkMAXExists(int deviceID) {
    if (sparkMaxes.containsKey(deviceID)) {
      throw new IllegalArgumentException("SparkMAX with deviceID " + deviceID + " already exists");
    }
  }

  private static void throwIfSparkMAXExists(SparkMax sparkMAX) {
    throwIfSparkMAXExists(sparkMAX.getDeviceId());
  }

  private static void throwIfSparkMAXDoesNotExist(int deviceID) {
    if (!sparkMaxes.containsKey(deviceID)) {
      throw new IllegalArgumentException("SparkMAX with deviceID " + deviceID + " does not exist");
    }
  }

  private static void throwIfSparkMAXDoesNotExist(SparkMax sparkMAX) {
    throwIfSparkMAXDoesNotExist(sparkMAX.getDeviceId());
  }

  public static void addSparkMAX(SparkMax sparkMAX) {
    throwIfSparkMAXExists(sparkMAX);
    sparkMaxes.put(sparkMAX.getDeviceId(), sparkMAX);
  }

  public static void removeSparkMAX(SparkMax sparkMAX) {
    throwIfSparkMAXDoesNotExist(sparkMAX);
    sparkMaxes.remove(sparkMAX.getDeviceId());
  }

  public static SparkMax getSparkMAX(int deviceID) {
    throwIfSparkMAXDoesNotExist(deviceID);
    return sparkMaxes.get(deviceID);
  }

  public static boolean doesSparkMAXExist(int deviceID) {
    return sparkMaxes.containsKey(deviceID);
  }

  /** Closes all SparkMAXes and clears the data. */
  public static void resetData() {
    for (SparkMax sparkMAX : sparkMaxes.values()) {
      sparkMAX.close();
    }
    sparkMaxes.clear();
  }
}
