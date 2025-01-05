package command_tests.utils;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

public class CommandTestBase {
  protected CommandScheduler commandScheduler;
  protected LEDSubsystem ledSubsystem;

  @BeforeEach
  protected void setUp() {
    assert HAL.initialize(500, 0);
    commandScheduler = CommandScheduler.getInstance();
    ledSubsystem = LEDSystem.getInstance();

    // Enable robot for commands to run
    DriverStationSim.setEnabled(true);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData(); // ! Breaks without this
  }

  @AfterEach
  protected void tearDown() {
    LEDSystem.resetLEDSubsystem();
    commandScheduler.cancelAll();
    commandScheduler.unregisterAllSubsystems(); // ! breaks all test tests if not done
    commandScheduler.close();
  }
}
