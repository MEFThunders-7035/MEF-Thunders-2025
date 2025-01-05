package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootToSpeakerCommand extends ParallelRaceGroup {
  private static final double waitTime = 2.5;

  public ShootToSpeakerCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    super(
        new SequentialCommandGroup(
            new BasicRunShooterCommand(shooterSubsystem, waitTime),
            new ParallelRaceGroup(shooterSubsystem.run(), intakeSubsystem.loadToShooter())));
  }

  public ShootToSpeakerCommand(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      ArmSubsystem armSubsystem,
      DriveSubsystem driveSubsystem) {
    super(
        // Constantly move the arm until all the other commands finish.
        armSubsystem.setArmToShooter(driveSubsystem::getDistanceToShooter),
        new ShootToSpeakerCommand(shooterSubsystem, intakeSubsystem));
  }

  public ShootToSpeakerCommand(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      ArmSubsystem armSubsystem,
      DriveSubsystem driveSubsystem,
      XboxController controller) {
    super(
        // Constantly move the arm until all the other commands finish.
        armSubsystem.setArmToShooter(driveSubsystem::getDistanceToShooter),
        DriveCommands.driveFacingShooter(driveSubsystem, controller),
        // The finishing command will be this one:
        new ShootToSpeakerCommand(shooterSubsystem, intakeSubsystem));
  }
}
