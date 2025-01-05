package wrapper_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.utils.sim_utils.ColorSensorV3Wrapped;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ColorSensorV3WrapperTests {
  private ColorSensorV3Wrapped colorSensor;

  @BeforeEach
  public void setUp() {
    HAL.initialize(500, 0); // init HAL just in case
    colorSensor = new ColorSensorV3Wrapped(Port.kMXP);
  }

  @AfterEach
  public void tearDown() {
    colorSensor.close();
  }

  @Test
  void testColorSensorV3Wrapping() {
    assertEquals(
        colorSensor, ColorSensorV3Wrapped.getColorSensor(), "ColorSensorV3 should be wrapped");
  }

  @Test
  void testColorSensorV3ValueChanging() {
    ColorSensorV3Wrapped.setRGBD(2500, 300, 100, 900);
    assertEquals(2500, colorSensor.getRed());
    assertEquals(300, colorSensor.getGreen());
    assertEquals(100, colorSensor.getBlue());
    assertEquals(900, colorSensor.getProximity());

    ColorSensorV3Wrapped.setRGBD(0, 0, 0, 0);
    assertEquals(0, colorSensor.getRed());
    assertEquals(0, colorSensor.getGreen());
    assertEquals(0, colorSensor.getBlue());
    assertEquals(0, colorSensor.getProximity());
  }
}
