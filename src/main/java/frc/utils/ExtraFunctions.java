package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.MagicConstants.ArmQuadraticFunction;

public class ExtraFunctions {
  /**
   * This is used to map a value from one range to another. Taken from:
   * https://www.arduino.cc/reference/en/language/functions/math/map/
   *
   * @param x the value to map
   * @param in_min the minimum value of the input range
   * @param in_max the maximum value of the input range
   * @param out_min the minimum value of the output range
   * @param out_max the maximum value of the output range
   * @return the mapped value
   */
  public static double mapValue(
      double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  public static int getShooterAprilTagID() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    return alliance == Alliance.Blue ? 7 : 4;
  }

  public static double getAngleFromDistance(double distance) {
    return ArmQuadraticFunction.kXSquared * (Math.pow(distance, 2))
        + ArmQuadraticFunction.kX * distance
        + ArmQuadraticFunction.kConstant;
  }

  public static Color getAllianceColor() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    return alliance == Alliance.Blue ? Color.kSkyBlue : Color.kFirstRed;
  }
}
