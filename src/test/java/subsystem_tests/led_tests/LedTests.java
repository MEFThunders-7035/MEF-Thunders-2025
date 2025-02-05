package subsystem_tests.led_tests;

import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static subsystem_tests.led_tests.utils.LEDTestUtils.checkForColorInAll;
import static subsystem_tests.led_tests.utils.LEDTestUtils.getColorAtIndex;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import subsystem_tests.SubsystemTestBase;

class LedTests extends SubsystemTestBase {
  private LEDSubsystem ledSubsystem;

  @BeforeEach
  @Override
  public void setUp() {
    super.setUp();
    ledSubsystem = new LEDSubsystem();
  }

  @AfterEach
  @Override
  public void tearDown() {
    super.tearDown();
    ledSubsystem.close();
  }

  private void testUntilPercentage(double percentage) {
    runCommand(ledSubsystem.runPattern(LEDPattern.progressMaskLayer(() -> percentage)));
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
    runCommand(ledSubsystem.runPattern(LEDPattern.solid(Color.kWhite)));
    Timer.delay(0.05); // let the loop change all the colors
    for (int i = 0; i < ledSubsystem.getLedCount(); i++) {
      assertEquals(
          Color.kWhite,
          getColorAtIndex(ledSubsystem, i),
          "Color Should be the same as filled color at led " + i);
    }
  }

  @Test
  void testBlink() {
    SimHooks.pauseTiming();
    SimHooks.restartTiming();
    SimHooks.stepTiming(Timer.getFPGATimestamp() % 0.5);
    runCommand(ledSubsystem.runPattern(LEDPattern.solid(Color.kWhite).blink(Seconds.of(0.5))));
    checkForColorInAll(
        ledSubsystem, Color.kWhite, "Starting color should have been the the color specified");
    SimHooks.stepTiming(0.5);
    commandScheduler.run(); // wait for led's to close back down
    checkForColorInAll(
        ledSubsystem, Color.kBlack, "Color should have turned to black in blinking sequence");
    SimHooks.stepTiming(0.5);
    commandScheduler.run();
    checkForColorInAll(
        ledSubsystem, Color.kWhite, "Color should have turned back to the color specified");
    SimHooks.resumeTiming();
  }

  @Test
  void testBlinkRed() {
    // We have to do this, because the blink command depends on the RobotController.getTime()
    // So to make it deterministic we have to pause the time and step it manually
    SimHooks.pauseTiming();
    SimHooks.restartTiming();
    SimHooks.stepTiming(Timer.getFPGATimestamp() % 0.5); // make sure we are in sync with the blink

    runCommand(ledSubsystem.runPattern(LEDPattern.solid(Color.kRed).blink(Seconds.of(0.5))));
    checkForColorInAll(ledSubsystem, Color.kRed, "Color should be red before blink");
    SimHooks.stepTiming(0.5);
    commandScheduler.run();
    checkForColorInAll(
        ledSubsystem, Color.kBlack, "Color should have turned to black in blinking sequence");
    SimHooks.stepTiming(0.5);
    commandScheduler.run();
    checkForColorInAll(ledSubsystem, Color.kRed, "Color should have turned back to red");

    // Let the rest of the tests work normally, by resuming the time
    SimHooks.resumeTiming();
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
}
