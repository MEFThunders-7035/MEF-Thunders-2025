package command_tests;

import static subsystem_tests.led_tests.utils.LEDTestUtils.checkForColorInAll;

import command_tests.utils.CommandTestBase;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.led_commands.LEDIdleCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.utils.sim_utils.ColorSensorV3Wrapped;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class LEDIdleCommandTest extends CommandTestBase {
  private IntakeSubsystem intakeSubsystem;

  @BeforeEach
  public void setUp() {
    super.setUp();
    HAL.initialize(500, 0);
    intakeSubsystem = new IntakeSubsystem();
  }

  @AfterEach
  public void tearDown() {
    super.tearDown();
    intakeSubsystem.close();
  }

  @Test
  void colorOnDisabled() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData(); // ! Breaks without this
    commandScheduler.schedule(
        new LEDIdleCommand(ledSubsystem, intakeSubsystem).ignoringDisable(true));
    commandScheduler.run();
    Timer.delay(0.1); // let led thread do its thing
    checkForColorInAll(ledSubsystem, Color.kOrangeRed, "Color Should be orange when disabled");
  }

  @Test
  void colorOnEnabled() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData(); // ! Breaks without this
    commandScheduler.schedule(
        new LEDIdleCommand(ledSubsystem, intakeSubsystem).ignoringDisable(true));

    // * check without Note
    ColorSensorV3Wrapped.setNoteColor(false);
    commandScheduler.run();
    Timer.delay(0.1); // let led thread do its thing
    checkForColorInAll(ledSubsystem, Color.kRed, "Color should be red when no Note is detected");

    // * check with Note
    ColorSensorV3Wrapped.setNoteColor(true);
    commandScheduler.run();
    Timer.delay(0.1); // let led thread do its thing
    checkForColorInAll(ledSubsystem, Color.kGreen, "Color should be green when Note is detected");
  }
}
