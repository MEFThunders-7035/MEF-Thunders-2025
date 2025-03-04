package frc.utils.sim_utils;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotBase;

public class ColorSensorV3Wrapped extends ColorSensorV3 implements AutoCloseable {
  private static ColorSensorV3 colorSensor;
  private static final int[] rgbd = new int[4]; // Red, Green, Blue, and Distance

  public ColorSensorV3Wrapped(I2C.Port port) {
    super(port);
    if (RobotBase.isReal()) {
      return;
    }
    // Do not check if colorSensor is null because if you do the unit test crashes for some reason.
    colorSensor = this;
  }

  @Override
  public void close() {
    if (RobotBase.isReal()) {
      return;
    }
    colorSensor = null;

    // Clear the RGBD values
    for (int i = 0; i < rgbd.length; i++) {
      rgbd[i] = 0;
    }
  }

  /**
   * This method is used for simulation purposes only. It returns the ColorSensorV3 singleton
   * instance
   *
   * @return The ColorSensorV3 singleton instance
   */
  public static ColorSensorV3 getColorSensor() {
    if (colorSensor == null) {
      throw new IllegalArgumentException("ColorSensorV3 does not exist");
    }
    return colorSensor;
  }

  public static void setRGBD(int red, int green, int blue, int distance) {
    if (RobotBase.isReal()) {
      return;
    }
    rgbd[0] = red;
    rgbd[1] = green;
    rgbd[2] = blue;
    rgbd[3] = distance;
  }

  public static void setNoteColor(boolean isNote, boolean isWhite) {
    if (isWhite) {
      setRGBD(1000, 1000, 1000, 1500); // Beyaz için R=G=B yüksek olmalı
    } else if (isNote) {
      setRGBD(1000, 300, 0, 1500); // Önceki nota algılama renkleri
    } else {
      setRGBD(0, 0, 0, 0); // Hiçbir şey algılanmıyor
    }
  }

  @Override
  public int getRed() {
    // we check if getRed() is 0 because the color sensor returns 0 when its not being "simulated"
    // as this is only used for unit tests.
    if (RobotBase.isReal() || colorSensor == null || super.getRed() != 0) {
      return super.getRed();
    }
    return rgbd[0];
  }

  @Override
  public int getGreen() {
    if (RobotBase.isReal() || colorSensor == null || super.getGreen() != 0) {
      return super.getGreen();
    }
    return rgbd[1];
  }

  @Override
  public int getBlue() {
    if (RobotBase.isReal() || colorSensor == null || super.getBlue() != 0) {
      return super.getBlue();
    }
    return rgbd[2];
  }

  @Override
  public int getProximity() {
    if (RobotBase.isReal() || colorSensor == null || super.getProximity() != 0) {
      return super.getProximity();
    }
    return rgbd[3];
  }
}
