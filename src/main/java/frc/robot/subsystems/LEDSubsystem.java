package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.utils.BetterLED;

public class LEDSubsystem extends SubsystemBase implements AutoCloseable {
  BetterLED strip;
  Color lastSetColor = new Color();
  Color lastSetBlinkingColor = new Color();
  boolean isBlinkingRed = false;

  public LEDSubsystem() {
    strip = new BetterLED(LEDConstants.kLedPin, LEDConstants.kLedCount);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("LED Currently Running", strip.getCurrentCommandName());
  }

  @Override
  public void close() {
    strip.close();
  }

  public int getLedCount() {
    return strip.getLedCount();
  }

  public void fill(Color color) {
    if (lastSetColor.equals(color)) {
      return;
    }
    strip.fillColor(color);
    lastSetColor = color;
    lastSetBlinkingColor = Color.kBlack;
    isBlinkingRed = false;
  }

  public void fill(Color color, int count) {
    fill(color, count, false);
  }

  public void fill(Color color, int count, boolean force) {
    if (lastSetColor.equals(color) && !force) {
      return;
    }
    strip.fillColor(color, count);
    lastSetColor = color;
    lastSetBlinkingColor = Color.kBlack;
  }

  public Command getRedBlinkCommand() {
    return this.run(this::blinkRed)
        .finallyDo(
            () -> {
              strip.removeFromLoop();
              lastSetBlinkingColor = new Color();
            });
  }

  public Command getBlinkColorCommand(Color color) {
    return this.run(() -> blink(color))
        .finallyDo(
            () -> {
              strip.removeFromLoop();
              lastSetBlinkingColor = new Color();
            });
  }

  public void setRed() {
    fill(Color.kRed);
  }

  public void setGreen() {
    fill(Color.kGreen);
  }

  public void setStatusColor(boolean status) {
    if (status) {
      setGreen();
    } else {
      setRed();
    }
  }

  public void blink(Color color, double delaySeconds) {
    if (lastSetBlinkingColor.equals(color)) {
      return;
    }
    lastSetColor = new Color(); // Reset back to 0
    lastSetBlinkingColor = color;
    strip.blink(color, delaySeconds);
  }

  public void blink(Color color) {
    blink(color, 0.2);
  }

  public void blinkRed() {
    if (isBlinkingRed) {
      return;
    }
    blink(new Color(255, 0, 0));
  }

  /**
   * Fills the led's until the given percentage.
   *
   * @param percentage a value between 0 and 1
   * @param color the color to fill with
   */
  public void fillPercentageWithColor(double percentage, Color color) {
    percentage = MathUtil.clamp(percentage, 0.0, 1.0);
    fill(color, (int) (strip.getLedCount() * percentage), true);
  }

  public BetterLED getStrip() {
    return strip;
  }
}
