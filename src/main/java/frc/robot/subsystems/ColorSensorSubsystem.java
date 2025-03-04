package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;
import frc.utils.sim_utils.ColorSensorV3Wrapped;

public class ColorSensorSubsystem extends SubsystemBase implements AutoCloseable {

  private final ColorSensorV3Wrapped colorSensor;

  public ColorSensorSubsystem() {
    colorSensor =
        new ColorSensorV3Wrapped(CoralIntakeConstants.ColorSensorConstants.kColorSensorPort);
  }

  @Override
  public void close() {
    colorSensor.close();
  }

  public boolean hasCoral() {
    int red = colorSensor.getRed();
    int blue = colorSensor.getBlue();
    // If we are really close, we will decrease the threshold.
    // Proximity is inversely proportional to distance. and max of 2047 is 0 inches.
    if (colorSensor.getProximity() > 1600) {
      red = (int) (red * (1600.0 / colorSensor.getProximity()));
      blue = (int) (blue * (1600.0 / colorSensor.getProximity())); // Missing semicolon fixed
    }

    // Define a condition to return whether an object (coral) is detected
    return red > blue; // Replace with actual detection logic based on color thresholds
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ColorSensor - Distance", colorSensor.getProximity());
    SmartDashboard.putNumber("ColorSensorAAA321 - Red", colorSensor.getRed());
    SmartDashboard.putNumber("ColorSensor - Green", colorSensor.getGreen());
    SmartDashboard.putNumber("ColorSensor - Blue", colorSensor.getBlue());
    SmartDashboard.putNumber("ColorSensor - IR", colorSensor.getIR());
    SmartDashboard.putBoolean("Note Detected", hasCoral());

    // Print out the color sensor values
    System.out.println(
        "Color Sensor Readings - Proximity: "
            + colorSensor.getProximity()
            + ", Red: "
            + colorSensor.getRed()
            + ", Green: "
            + colorSensor.getGreen()
            + ", Blue: "
            + colorSensor.getBlue()
            + ", IR: "
            + colorSensor.getIR()
            + ", Note Detected: "
            + hasCoral());

    Shuffleboard.getTab("Color Sensor")
        .add("Proximity", colorSensor.getProximity())
        .withWidget("Graph");
  }
}
