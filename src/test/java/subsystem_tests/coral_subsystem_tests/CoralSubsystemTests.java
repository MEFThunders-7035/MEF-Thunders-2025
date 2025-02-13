package subsystem_tests.coral_subsystem_tests;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.simulation.UltrasonicSim;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.subsystems.CoralSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import subsystem_tests.SubsystemTestBase;

class CoralSubsystemTests extends SubsystemTestBase {
  private CoralSubsystem coralSubsystem;
  private UltrasonicSim ultrasonicSim;

  @BeforeEach
  @Override
  public void setUp() {
    super.setUp();
    coralSubsystem = new CoralSubsystem();
    ultrasonicSim =
        new UltrasonicSim(
            CoralIntakeConstants.kUltrasonicPingChannel,
            CoralIntakeConstants.kUltrasonicEchoChannel);
  }

  @AfterEach
  @Override
  public void tearDown() {
    super.tearDown();
    coralSubsystem.close();
  }

  @Test
  void testItStops() {
    ultrasonicSim.setRangeMeters(0.05); // 50mm
    assertFalse(coralSubsystem.hasCoral());

    var command = coralSubsystem.takeCoral();
    runCommand(command);

    ultrasonicSim.setRangeMeters(0.02); // 20mm
    assertTrue(coralSubsystem.hasCoral());

    commandScheduler.run();
    assertFalse(command.isScheduled());
  }
}
