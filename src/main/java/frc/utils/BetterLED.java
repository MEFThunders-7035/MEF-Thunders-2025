package frc.utils;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import java.util.ArrayList;
import java.util.concurrent.locks.ReentrantLock;

public class BetterLED extends AddressableLED {
  public enum AnimationType { // Is already static.
    LEFT_TO_RIGHT,
    RIGHT_TO_LEFT,
    MIDDLE_TO_ENDS,
    ENDS_TO_MIDDLE
  }

  private AddressableLEDBuffer mainBuffer;
  private int ledCount;
  private int rainbowFirstPixelHue = 0;
  private int currentAnimationIndex = 0;

  private final ReentrantLock ledMutex = new ReentrantLock(true);
  private volatile boolean isThreadKilled = false;

  // The call list will have all animation calls in it,
  // This will run like a scheduler, just on a smaller scale.
  // it will probably have only a single function, but is an Arraylist just in case.
  private final ArrayList<LEDCommand> commandList;

  // The Thread will handle calling the callList and all other requirements for animations.
  private Thread ledUpdateThread;

  public BetterLED(int port, int ledCount) {
    super(port);
    mainBuffer = new AddressableLEDBuffer(ledCount);
    this.setLength(mainBuffer.getLength());
    this.start();
    commandList = new ArrayList<>(10); // 3 should be enough on most cases.
    // Create an update thread that handles animations
    setupThread();
    stopAnimation();
  }

  private void setupThread() {
    isThreadKilled = false; // Reset the kill flag on setup of thread
    ledUpdateThread =
        new Thread(
            () -> {
              do { // Infinity call periodic,
                ledMutex.lock();
                try {
                  periodic();
                } finally {
                  ledMutex.unlock();
                }
                Timer.delay(0.015);

                // Kill the thread if it has been set to kill.
              } while (!isThreadKilled);
            },
            "LED Control Thread");
    ledUpdateThread.setDaemon(true);
    ledUpdateThread.start();
  }

  @Override
  public void close() {
    isThreadKilled = true;
    super.close();
  }

  /**
   * Check if the thread is alive and if not, restart it.
   *
   * @return thread got restarted
   *     <p>true if restarted, false if not
   */
  public boolean checkThreadStatus() {
    if (!ledUpdateThread.isAlive()) {
      setupThread(); // restart thread
      return true;
    }
    return false;
  }

  /**
   * Returns the buffer thats used. You should never need to use this except debugging or similar.
   *
   * @return The main buffer used by the LED instance.
   */
  public AddressableLEDBuffer getBuffer() {
    return mainBuffer;
  }

  public int getLedCount() {
    return ledCount;
  }

  public void fillColor(Color color, int start, int end) {
    addToLoop(
        new LEDCommand(() -> fill(color, start, end), false, "Fill with: " + color.toHexString()));
  }

  public void fillColor(Color color, int end) {
    fillColor(color, 0, end);
  }

  public void fillColor(Color color) {
    fillColor(color, ledCount);
  }

  private void fill(Color color, int start, int end) {
    for (int i = start; i < end; i++) {
      mainBuffer.setLED(i, color);
    }
    this.setData(mainBuffer);
  }

  private void fill(Color color, int end) {
    fill(color, 0, end);
  }

  private void fill(Color color) {
    fill(color, ledCount);
  }

  public void startRainbow() {
    changeLoopTo(new LEDCommand(this::rainbowLoop, true, "Rainbow"));
  }

  public void animateColor(Color color, AnimationType animation) {
    animateColor(color, animation, (ledCount * 2) / 3);
  }

  public void animateColor(Color color, AnimationType animation, int onLEDCount) {
    switch (animation) {
      case LEFT_TO_RIGHT:
        changeLoopTo(createLeftToRightAnimationLoop(color, onLEDCount));
        break;
      case RIGHT_TO_LEFT:
        changeLoopTo(createRightToLeftAnimationLoop(color, onLEDCount));
        break;
      case MIDDLE_TO_ENDS:
        break; // TODO: implement
      case ENDS_TO_MIDDLE:
        break; // TODO: implement
    }
  }

  public void stopAnimation() {
    commandList.clear();
  }

  public void blink(Color color, int blinkCount) {
    changeLoopTo(
        new LEDCommand(
            () -> {
              fill(color);
              Timer.delay(0.5);
              fill(Color.kBlack);
              Timer.delay(0.5);
            },
            blinkCount,
            "blink %d times".formatted(blinkCount)));
  }

  public void blink(Color color, double delaySeconds) {
    changeLoopTo(
        new LEDCommand(
            () -> {
              fill(color);
              Timer.delay(delaySeconds);
              fill(Color.kBlack);
              Timer.delay(delaySeconds);
            },
            true,
            "blink %s infinitely".formatted(color.toString())));
  }

  public void blink(Color color) {
    blink(color, 0.2);
  }

  /**
   * Breathe the led, this will light up the led and then light it down. This will repeat forever.
   *
   * @param color The color to breathe. see: WPILIB {@link Color}
   * @param delayTime The time delay between all the steps in seconds; NOT the total time!
   * @param max The max value of the led, normally the maximum is 255.
   * @param min The min value of the led, normally the minimum is 0.
   */
  public void breathe(Color color, double delayTime, int max, int min) {
    changeLoopTo(
        new LEDCommand(
            () -> {
              // Light UP The led
              for (int i = min; i < max; i++) {
                fill(new Color(color.red * i, color.green * i, color.blue * i));
                Timer.delay(delayTime);
              }
              // Light DOWN the led
              for (int i = max; i > min; i--) {
                fill(new Color(color.red * i, color.green * i, color.blue * i));
                Timer.delay(delayTime);
              }
            },
            true,
            "breath %s".formatted(color.toString())));
  }

  public void breathe(Color color, double delayTime) {
    breathe(color, delayTime, 255, 0);
  }

  public void breathe(Color color) {
    breathe(color, 0.01);
  }

  @Override
  public void setLength(int length) {
    mainBuffer = new AddressableLEDBuffer(length);
    ledCount = length;
    super.setLength(length);
  }

  private void changeLoopTo(LEDCommand func) {
    ledMutex.lock();
    try {
      commandList.clear(); // Clear the list first
      commandList.add(func); // then add the runnable
    } finally {
      ledMutex.unlock();
    }
  }

  public void addToLoop(LEDCommand command) {
    ledMutex.lock();
    try {
      commandList.add(command);
    } finally {
      ledMutex.unlock();
    }
  }

  public void removeFromLoop(int amount) {
    ledMutex.lock();
    try {
      for (int i = 0; i < amount; i++) {
        if (!commandList.isEmpty()) {
          commandList.remove(0);
        }
      }
    } finally {
      ledMutex.unlock();
    }
  }

  public void removeFromLoop() {
    removeFromLoop(1);
  }

  private void periodic() {
    // Call any functions in the callList
    if (commandList.isEmpty()) {
      return;
    }
    var command = commandList.get(0);
    DataLogManager.log(command.getName());
    command.execute();
    if (command.hasEnded()) {
      commandList.remove(0);
    }
  }

  private void rainbowLoop() {
    for (int i = 0; i < ledCount; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledCount)) % 80;
      // Set the value
      mainBuffer.setHSV(i, hue, 255, 255);
    }
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
    this.setData(mainBuffer);
  }

  private LEDCommand createLeftToRightAnimationLoop(Color color, int onLEDCount) {
    return new LEDCommand(
        () -> {
          final int start =
              currentAnimationIndex > onLEDCount ? currentAnimationIndex - onLEDCount : 0;
          for (int i = start; i < ledCount && i < currentAnimationIndex; i++) {
            mainBuffer.setLED(i, color);
          }
          currentAnimationIndex++;
          currentAnimationIndex %= ledCount;
          this.setData(mainBuffer);
          Timer.delay(0.1);
        },
        true,
        "Left To Right %s".formatted(color.toString()));
  }

  private LEDCommand createRightToLeftAnimationLoop(Color color, int onLEDCount) {
    return new LEDCommand(
        () -> {
          final int start =
              currentAnimationIndex < onLEDCount ? currentAnimationIndex + onLEDCount : ledCount;
          for (int i = start; i > 0 && i > currentAnimationIndex; i--) {
            mainBuffer.setLED(i, color);
          }
          currentAnimationIndex--;
          currentAnimationIndex =
              Math.floorMod(
                  currentAnimationIndex, ledCount); // ! DO NOT USE % WITH NEGATIVE NUMBERS.
          this.setData(mainBuffer);
          Timer.delay(0.1);
        },
        true,
        "Right to Left %s".formatted(color.toString()));
  }

  public String getCurrentCommandName() {
    if (commandList.isEmpty()) {
      return "No Command Running";
    }
    return commandList.get(0).toString();
  }
}
