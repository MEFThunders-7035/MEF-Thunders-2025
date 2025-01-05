package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.function.BooleanSupplier;

public class WaitANDConditionCommand extends WaitCommand {
  private final BooleanSupplier condition;

  public WaitANDConditionCommand(double seconds, BooleanSupplier condition) {
    super(seconds);
    this.condition = condition;
  }

  @Override
  public boolean isFinished() {
    return super.isFinished() && condition.getAsBoolean();
  }
}
