package command_tests.arm_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static subsystem_tests.led_tests.utils.LEDTestUtils.testAtTime;

import command_tests.utils.CommandTestBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.BasicRunShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class BasicShooterTest extends CommandTestBase {
  private ShooterSubsystem shooterSubsystem;
  private PWMSim shooterMotor;

  private static final double kWaitTime = 2;

  @BeforeEach
  public void setUp() {
    super.setUp();
    shooterSubsystem = new ShooterSubsystem();
    shooterMotor = new PWMSim(ShooterConstants.kShooterMotorPwmID);
    BasicRunShooterCommand basicShooterCommand =
        new BasicRunShooterCommand(shooterSubsystem, kWaitTime);

    commandScheduler.schedule(basicShooterCommand);
  }

  @AfterEach
  public void tearDown() {
    super.tearDown();
    shooterSubsystem.close();
  }

  @Test
  void testSetSpeed() {
    commandScheduler.run();
    // Motor should be at the set speed
    assertEquals(
        ShooterConstants.kShooterSpeed,
        shooterMotor.getSpeed(),
        "Shooter motor should be at shooter speed");
    SimHooks.stepTiming(kWaitTime);
    commandScheduler.run();
    // Motor should be at 0 after waitTime
    assertEquals(
        0,
        shooterMotor.getSpeed(),
        "Shooter motor should be at 0 after %s seconds".formatted(kWaitTime));
  }

  @Test
  void testLEDLoading() {
    double startTime = Timer.getFPGATimestamp();
    commandScheduler.run();
    SimHooks.stepTiming(kWaitTime); // load waitTime
    commandScheduler.run();
    testAtTime(ledSubsystem, startTime, kWaitTime);
  }
}
