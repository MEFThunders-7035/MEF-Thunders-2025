package subsystem_tests.led_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static subsystem_tests.led_tests.utils.LEDTestUtils.checkForColorInAll;
import static subsystem_tests.led_tests.utils.LEDTestUtils.getColorAtIndex;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class LedTests {
  private LEDSubsystem ledSubsystem;

  @BeforeEach
  public void setUp() {
    HAL.initialize(500, 0);
    ledSubsystem = LEDSystem.getInstance();
  }

  @AfterEach
  public void tearDown() {
    // ledSubsystem is closed by `LEDSystem.resetLEDSubsystem()`
    LEDSystem.resetLEDSubsystem();
  }

  private void testUntilPercentage(double percentage) {
    ledSubsystem.fillPercentageWithColor(percentage, Color.kWhite);
    Timer.delay(0.1); // wait until command gets executed
    for (int i = 0; i < ledSubsystem.getLedCount(); i++) {
      if (i < (int) (ledSubsystem.getLedCount() * percentage)) {
        assertEquals(
            Color.kWhite,
            getColorAtIndex(ledSubsystem, i),
            "Color Should be the specified one until " + percentage);
      } else {
        assertEquals(
            Color.kBlack,
            getColorAtIndex(ledSubsystem, i),
            "Color Should be Black after the specified percentage");
      }
    }
  }

  @Test
  void testFill() {
    ledSubsystem.fill(Color.kWhite);
    Timer.delay(0.05); // let the loop change all the colors
    for (int i = 0; i < ledSubsystem.getStrip().getLedCount(); i++) {
      assertEquals(
          Color.kWhite,
          getColorAtIndex(ledSubsystem, i),
          "Color Should be the same as filled color at led " + i);
    }
  }

  @Test
  void testBlink() {
    ledSubsystem.blink(Color.kWhite, 0.5);
    Timer.delay(0.1);
    checkForColorInAll(
        ledSubsystem, Color.kWhite, "Starting color should have been the the color specified");
    Timer.delay(0.5); // wait for led's to close back down
    checkForColorInAll(
        ledSubsystem, Color.kBlack, "Color should have turned to black in blinking sequence");
    Timer.delay(0.5);
    checkForColorInAll(
        ledSubsystem, Color.kWhite, "Color should have turned back to the color specified");
  }

  @Test
  void testBlinkRed() {
    ledSubsystem.blinkRed();
    Timer.delay(0.1);
    checkForColorInAll(ledSubsystem, Color.kRed, "Color should be red before blink");
    Timer.delay(0.2);
    checkForColorInAll(
        ledSubsystem, Color.kBlack, "Color should have turned to black in blinking sequence");
    Timer.delay(0.2);
    checkForColorInAll(ledSubsystem, Color.kRed, "Color should have turned back to red");
  }

  @Test
  void testFillPercentage() {
    testUntilPercentage(0.1);
    tearDown();

    setUp();
    testUntilPercentage(0.2);
    tearDown();

    setUp();
    testUntilPercentage(0.3);
    tearDown();

    setUp();
    testUntilPercentage(0.4);
    tearDown();

    setUp();
    testUntilPercentage(0.6);
    tearDown();

    setUp();
    testUntilPercentage(0.7);
    tearDown();

    setUp();
    testUntilPercentage(0.8);
    tearDown();

    setUp();
    testUntilPercentage(0.9);
    // Auto Teardown
  }

  @Test
  void testStatusFills() {
    ledSubsystem.setStatusColor(false);
    Timer.delay(0.1);
    checkForColorInAll(ledSubsystem, Color.kRed, "Color Should be red when status is false");
    tearDown();

    setUp();
    ledSubsystem.setStatusColor(true);
    Timer.delay(0.1);
    checkForColorInAll(ledSubsystem, Color.kGreen, "Color should be green when status is true");
  }
}
