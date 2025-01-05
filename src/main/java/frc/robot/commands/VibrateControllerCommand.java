package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.util_commands.RepeatForCommand;

public class VibrateControllerCommand extends SequentialCommandGroup {
  public VibrateControllerCommand(
      XboxController controller, int repetitions, double intensity, double duration) {
    super(createVibrateControllerCommand(controller, repetitions, intensity, duration));
  }

  public VibrateControllerCommand(XboxController controller, int repetitions, double intensity) {
    this(controller, repetitions, intensity, 0.5);
  }

  public VibrateControllerCommand(XboxController controller, int repetitions) {
    this(controller, repetitions, 1);
  }

  private static Command createVibrateControllerCommand(
      XboxController controller, int repetitions, double intensity, double duration) {

    SequentialCommandGroup toRepeatCommand =
        new SequentialCommandGroup(
            new InstantCommand(() -> controller.setRumble(RumbleType.kBothRumble, intensity)), //
            new WaitCommand(duration),
            new InstantCommand(() -> controller.setRumble(RumbleType.kBothRumble, 0)),
            new WaitCommand(duration));

    return new RepeatForCommand(toRepeatCommand, repetitions)
        .finallyDo(() -> controller.setRumble(RumbleType.kBothRumble, 0));
  }
}
