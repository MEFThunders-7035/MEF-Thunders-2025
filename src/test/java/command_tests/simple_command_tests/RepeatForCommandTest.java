package command_tests.simple_command_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import command_tests.utils.CommandTestBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.util_commands.RepeatForCommand;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class RepeatForCommandTest extends CommandTestBase {
  // TODO: setup repeatForCommand with multiple command types, and test them respectively.
  private int counter = 0;

  @BeforeEach
  public void setUp() {
    super.setUp();
  }

  @AfterEach
  public void tearDown() {
    super.tearDown();
  }

  @Test
  void testRepeat() {
    counter = 0;
    RepeatForCommand repeatForCommand =
        new RepeatForCommand(new InstantCommand(() -> counter++), 10);
    commandScheduler.schedule(repeatForCommand);

    while (commandScheduler.isScheduled(repeatForCommand)) {
      commandScheduler.run();
    }

    assertEquals(10, counter);
  }
}
