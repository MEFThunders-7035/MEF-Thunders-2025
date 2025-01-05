package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public final class LEDSystem {
  private static LEDSubsystem ledSubsystemInstance = new LEDSubsystem();

  private LEDSystem() {}

  public static LEDSubsystem getInstance() {
    return ledSubsystemInstance;
  }

  public static Command run(Runnable toRun) {
    return ledSubsystemInstance.run(toRun);
  }

  public static Command getBlinkRedCommand() {
    return ledSubsystemInstance.getRedBlinkCommand();
  }

  public static Command getBlinkColorCommand(Color color) {
    return ledSubsystemInstance.getBlinkColorCommand(color);
  }

  public static void resetLEDSubsystem() {
    if (ledSubsystemInstance == null) {
      ledSubsystemInstance = new LEDSubsystem();
      return;
    }
    ledSubsystemInstance.close();
    ledSubsystemInstance = new LEDSubsystem();
  }

  public static void changeLEDSubsystemInstance(LEDSubsystem ledSubsystem) {
    ledSubsystemInstance = ledSubsystem;
  }
}
