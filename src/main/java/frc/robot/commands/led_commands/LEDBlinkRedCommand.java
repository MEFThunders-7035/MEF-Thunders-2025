package frc.robot.commands.led_commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LEDSubsystem;

public class LEDBlinkRedCommand extends SequentialCommandGroup {
  public LEDBlinkRedCommand(LEDSubsystem ledSubsystem) {
    super(ledSubsystem.getRedBlinkCommand());
  }
}
