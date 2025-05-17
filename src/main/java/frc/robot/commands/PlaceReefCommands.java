package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class PlaceReefCommands {
  public static Command L1(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
    return Commands.sequence(
        armSubsystem.set(ArmPosition.IDLE), elevatorSubsystem.set(ElevatorPosition.L1));
  }

  public static Command L2(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
    return Commands.sequence(
        armSubsystem.set(ArmPosition.L2), elevatorSubsystem.set(ElevatorPosition.L2));
  }

  public static Command L3(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
    return Commands.sequence(
        armSubsystem.set(ArmPosition.L3), elevatorSubsystem.set(ElevatorPosition.L3));
  }

  public static Command L4(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
    return Commands.sequence(
        armSubsystem.set(ArmPosition.L4), elevatorSubsystem.set(ElevatorPosition.L4));
  }
}
