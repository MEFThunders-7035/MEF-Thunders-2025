package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.ExtraFunctions;

public class ShootToAmpCommand extends SequentialCommandGroup {
  public ShootToAmpCommand(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      ArmSubsystem armSubsystem) {
    super(
        new ParallelRaceGroup(
            armSubsystem.setArmToAmp(),
            shooterSubsystem.run(),
            LEDSystem.getBlinkColorCommand(ExtraFunctions.getAllianceColor())),
        new ParallelRaceGroup(
            intakeSubsystem.loadToShooter(),
            shooterSubsystem.run(),
            LEDSystem.getBlinkColorCommand(Color.kGreen)));
  }
}
