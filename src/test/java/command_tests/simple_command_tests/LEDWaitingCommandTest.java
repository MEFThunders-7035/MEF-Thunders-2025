package command_tests.simple_command_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static subsystem_tests.led_tests.utils.LEDTestUtils.testAtTime;

import command_tests.utils.CommandTestBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class LEDWaitingCommandTest extends CommandTestBase {
  private static final double kWaitTime = 2.0;
  private LEDSubsystem ledSubsystem;
  private Command ledLoadingWaitCommand;

  @BeforeEach
  @Override
  public void setUp() {
    super.setUp();
    ledSubsystem = new LEDSubsystem();
    ledLoadingWaitCommand = ledSubsystem.loadingWaitCommand(kWaitTime, LEDSubsystem.DEFAULT_COLOR);
    commandScheduler.schedule(ledLoadingWaitCommand);
  }

  @AfterEach
  @Override
  public void tearDown() {
    super.tearDown();
    ledSubsystem.close();
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
      commandScheduler.schedule(ledSubsystem.loadingWaitCommand(i, LEDSubsystem.DEFAULT_COLOR));
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
    commandScheduler.run();
    commandScheduler.run();
    assertEquals(
        true, ledLoadingWaitCommand.isFinished(), "led wait command should finish after wait time");
  }
}
