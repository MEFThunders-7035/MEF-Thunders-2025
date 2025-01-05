package command_tests.arm_tests;

import static org.junit.jupiter.api.Assertions.assertEquals;

import command_tests.utils.CommandTestBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ShootToAmpCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShootToAmpTest extends CommandTestBase {

  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private ArmSubsystem armSubsystem;
  private ShootToAmpCommand shootToAmpCommand;

  @BeforeEach
  @Override
  public void setUp() {
    super.setUp();

    shooterSubsystem = new ShooterSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    armSubsystem = new ArmSubsystem();

    shootToAmpCommand = new ShootToAmpCommand(shooterSubsystem, intakeSubsystem, armSubsystem);

    commandScheduler.schedule(shootToAmpCommand);
  }

  @AfterEach
  @Override
  public void tearDown() {
    super.tearDown();

    shooterSubsystem.close();
    intakeSubsystem.close();
    armSubsystem.close();
  }

  @Test
  void testItMovesToAmp() {
    commandScheduler.run();
    commandScheduler.run();

    assertEquals(ArmConstants.AMP_POSITION, armSubsystem.getDesiredPosition());
  }
}
