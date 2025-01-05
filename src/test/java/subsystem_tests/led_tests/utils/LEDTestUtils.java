package subsystem_tests.led_tests.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.led_commands.LEDLoadingWaitCommand;
import frc.robot.subsystems.LEDSubsystem;

public class LEDTestUtils {
  public static void checkForColorInAll(
      LEDSubsystem ledSubsystem, Color colorItShouldBe, int untilIndex, String message) {
    for (int i = 0; i < untilIndex; i++) {
      assertEquals(colorItShouldBe, getColorAtIndex(ledSubsystem, i), message + " at index: " + i);
    }
  }

  public static void checkForColorInAll(
      LEDSubsystem ledSubsystem, Color colorItShouldBe, String message) {
    checkForColorInAll(ledSubsystem, colorItShouldBe, ledSubsystem.getLedCount(), message);
  }

  /**
   * Auto Calculates the end time
   *
   * @param ledSubsystem
   * @param startTime
   * @param waitTime
   */
  public static void testAtTime(LEDSubsystem ledSubsystem, double startTime, double waitTime) {
    double endTime = Timer.getFPGATimestamp();
    int untilIndex =
        (int)
            (ledSubsystem.getLedCount()
                * MathUtil.clamp((endTime - startTime) / waitTime, 0.0, 1.0));
    Timer.delay(0.1); // let led loop do its thing
    checkForColorInAll(
        ledSubsystem,
        LEDLoadingWaitCommand.DEFAULT_COLOR,
        untilIndex,
        "Color should be default color until %s".formatted(untilIndex));
  }

  public static Color getColorAtIndex(LEDSubsystem ledSubsystem, int index) {
    return ledSubsystem.getStrip().getBuffer().getLED(index);
  }
}
