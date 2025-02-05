package frc.robot.commands.led_commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LEDSubsystem;

public class LEDBlinkRedCommand extends SequentialCommandGroup {
  public LEDBlinkRedCommand(LEDSubsystem ledSubsystem) {
    super(ledSubsystem.runPattern(LEDPattern.solid(Color.kRed).blink(Seconds.of(0.5))));
  }
}
