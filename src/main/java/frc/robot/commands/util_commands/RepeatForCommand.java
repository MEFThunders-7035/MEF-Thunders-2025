package frc.robot.commands.util_commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

public class RepeatForCommand extends RepeatCommand {
  private double count;
  private Command command;

  /**
   * Repeats a command for a specified number of times
   *
   * @param command The command to repeat
   * @param count The number of times to repeat the command
   */
  public RepeatForCommand(Command command, int count) {
    super(command);
    this.command = command;
    this.count = count;
  }

  @Override
  public void execute() {
    // only decrement the count if the command has finished
    if (command.isFinished()) {
      count--;
    }
    super.execute();
  }

  @Override
  public boolean isFinished() {
    return count <= 0;
  }
}
