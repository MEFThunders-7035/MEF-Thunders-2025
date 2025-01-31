package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase implements AutoCloseable {
  public static final Color DEFAULT_COLOR = new Color(0, 200, 255);

  private final AddressableLED led;
  private final AddressableLEDBuffer mainBuffer;

  public LEDSubsystem() {
    led = new AddressableLED(LEDConstants.kLedPin);
    mainBuffer = new AddressableLEDBuffer(LEDConstants.kLedCount);
    led.setLength(mainBuffer.getLength());
    led.start();
  }

  @Override
  public void periodic() {
    led.setData(mainBuffer);
  }

  @Override
  public void close() {
    led.close();
  }

  public int getLedCount() {
    return mainBuffer.getLength();
  }

  public Command runPattern(LEDPattern pattern) {
    return this.run(() -> pattern.applyTo(mainBuffer));
  }

  public Command loadingWaitCommand(double waitTime, Color color) {
    var timer = new Timer();
    timer.start();
    var base = LEDPattern.solid(color);
    var mask = LEDPattern.progressMaskLayer(() -> timer.get() / waitTime);
    var pattern = base.mask(mask);
    return this.runOnce(timer::reset)
        .andThen(runPattern(pattern))
        .until(() -> timer.get() > waitTime);
  }

  // Only for Simulation
  public Color getColorAtIndex(int index) {
    return mainBuffer.getLED(index);
  }
}
