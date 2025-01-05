package subsystem_tests.drive_subsystem_tests.utils;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;

public class NavXSim {
  public static void setAngle(double yaw) {
    int device = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(device, "Yaw"));
    angle.set(yaw);
  }

  public static void setConnected(boolean connected) {
    int device = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimBoolean isConnected =
        new SimBoolean(SimDeviceDataJNI.getSimValueHandle(device, "Connected"));
    isConnected.set(connected);
  }
}
