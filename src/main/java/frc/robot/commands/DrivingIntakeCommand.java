import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class DrivingIntakeCommand extends ParallelRaceGroup {
  public DrivingIntakeCommand(
      IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, XboxController controller) {
    super(intakeSubsystem.run(), DriveCommands.driveWithController(driveSubsystem, controller));
  }
}
