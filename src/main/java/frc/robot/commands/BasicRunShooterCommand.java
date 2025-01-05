package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.led_commands.LEDLoadingWaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class BasicRunShooterCommand extends ParallelRaceGroup {
  public BasicRunShooterCommand(ShooterSubsystem shooterSubsystem, double untilTimeSeconds) {
    super(shooterSubsystem.run(), new LEDLoadingWaitCommand(untilTimeSeconds));
  }
}
