package command_tests.simple_command_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static subsystem_tests.led_tests.utils.LEDTestUtils.testAtTime;

import command_tests.utils.CommandTestBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.commands.led_commands.LEDLoadingWaitCommand;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class LEDWaitingCommandTest extends CommandTestBase {
  private static final double kWaitTime = 2.0;
  private LEDLoadingWaitCommand ledLoadingWaitCommand;

  @BeforeEach
  public void setUp() {
    super.setUp();
    ledLoadingWaitCommand = new LEDLoadingWaitCommand(kWaitTime);
    commandScheduler.schedule(ledLoadingWaitCommand);
  }

  @AfterEach
  public void tearDown() {
    super.tearDown();
  }

  void testLEDLoading(double waitTime) {
    double startTime = Timer.getFPGATimestamp();
    for (double i = 0.1; Timer.getFPGATimestamp() - startTime + i < waitTime; i += 0.1) {
      SimHooks.stepTiming(i);
      commandScheduler.run();
      testAtTime(ledSubsystem, startTime, waitTime);
    }
  }

  /**
   * This test takes a lot of time as it tries a lot of differently timed {@link
   * LEDLoadingWaitCommand}s
   */
  @Test
  void testLEDLoading() {
    // try multiple wait times, just so we can do it
    for (double i = 0.0; i <= 6; i += 1.2) {
      commandScheduler.cancelAll();
      commandScheduler.schedule(new LEDLoadingWaitCommand(i));
      commandScheduler.run();
      testLEDLoading(i);
    }
  }

  @Test
  void itActuallyEnds() {
    commandScheduler.run();
    assertEquals(
        false,
        ledLoadingWaitCommand.isFinished(),
        "led wait command shouldn't finish before wait time");
    SimHooks.stepTiming(kWaitTime);
    assertEquals(
        true, ledLoadingWaitCommand.isFinished(), "led wait command should finish after wait time");
  }
}
