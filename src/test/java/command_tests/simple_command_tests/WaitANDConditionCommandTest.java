package command_tests.simple_command_tests;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import command_tests.utils.CommandTestBase;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.commands.WaitANDConditionCommand;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class WaitANDConditionCommandTest extends CommandTestBase {
  private WaitANDConditionCommand waitConditionCommand;
  private static double kWaitTime = 2.0;
  private boolean condition;

  @BeforeEach
  public void setUp() {
    super.setUp();

    condition = false;

    waitConditionCommand = new WaitANDConditionCommand(kWaitTime, () -> condition);
    commandScheduler.schedule(waitConditionCommand);
  }

  @AfterEach
  public void tearDown() {
    super.tearDown();
  }

  @Test
  void testItCompletesOnCondition() {
    commandScheduler.run();
    condition = true;
    commandScheduler.run();
    assertFalse(
        waitConditionCommand.isFinished(),
        "command should not finish after condition true without time finish");
  }

  @Test
  void testItCompletesAfterTime() {
    commandScheduler.run();
    SimHooks.stepTiming(kWaitTime);
    commandScheduler.run();
    assertFalse(
        waitConditionCommand.isFinished(),
        "Command should not finish after time finishes without condition");
  }

  @Test
  void testItFinishesAfterAll() {
    commandScheduler.run();
    condition = true;
    SimHooks.stepTiming(kWaitTime);
    commandScheduler.run();
    assertTrue(waitConditionCommand.isFinished(), "Command should finish after time and condition");
  }
}
