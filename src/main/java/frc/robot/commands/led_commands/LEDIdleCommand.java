package frc.robot.commands.led_commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class LEDIdleCommand extends RunCommand {

  public LEDIdleCommand(LEDSubsystem ledSubsystem, IntakeSubsystem intakeSubsystem) {
    super(() -> toDo(ledSubsystem, intakeSubsystem), ledSubsystem);
  }

  private static void toDo(LEDSubsystem ledSubsystem, IntakeSubsystem intakeSubsystem) {
    if (DriverStation.isEnabled()) {
      ledSubsystem.setStatusColor(intakeSubsystem.hasNote());
    } else {
      ledSubsystem.fill(Color.kOrangeRed);
    }
  }
}
