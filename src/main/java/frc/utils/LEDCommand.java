package frc.utils;

public class LEDCommand {
  private Runnable taskToRun;
  private boolean isMultiRun;
  private String commandName;
  private int runTimes = -1;

  /**
   * Creates a command type that will run once or infinitely
   *
   * @param toRun
   * @param isMultiRun
   */
  public LEDCommand(Runnable toRun, boolean isMultiRun, String name) {
    this.taskToRun = toRun;
    this.isMultiRun = isMultiRun;
    this.commandName = name;
  }

  /**
   * Creates a command type that will be run by the specified amount
   *
   * @param toRun
   * @param RunTimes will be run by the specified amount
   */
  public LEDCommand(Runnable toRun, int runTimes, String name) {
    this.taskToRun = toRun;
    this.runTimes = runTimes;
    this.commandName = name;

    // a command with runTimes is always multiRun
    this.isMultiRun = true;
  }

  public boolean hasEnded() {
    if (!isMultiRun) return true;

    // has run the specified amount
    if (runTimes == 0) return true;

    // reduce the amount of runTimes
    if (runTimes > 0) runTimes--;

    // Continue
    return false;
  }

  public void execute() {
    taskToRun.run();
  }

  public String getName() {
    return commandName;
  }

  @Override
  public String toString() {
    return getName();
  }
}
