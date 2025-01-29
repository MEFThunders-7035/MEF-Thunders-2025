package subsystem_tests.drive_subsystem_tests.utils;

import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class NavXSim {
  // Taken and Modified from:
  // https://github.com/Studica-Robotics/NavX
  public static void setAngle(double yaw) {
    var device = new SimDeviceSim("navX-Sensor", 4);
    var angle = device.getDouble("Yaw");
    angle.set(yaw);
  }

  public static void setConnected(boolean connected) {
    var device = new SimDeviceSim("navX-Sensor", 4);
    var isConnected = device.getBoolean("Connected");
    isConnected.set(connected);
  }
}
