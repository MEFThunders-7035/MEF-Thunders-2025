package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmEncoderSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutomaticArmElevator extends Command {
  private final AlgaeArmEncoderSubsystem algaeArm;
  private final ElevatorSubsystem elevator;
  private double lastElevatorPosition;

  public AutomaticArmElevator(AlgaeArmEncoderSubsystem algaeArm, ElevatorSubsystem elevator) {
    this.algaeArm = algaeArm;
    this.elevator = elevator;
    addRequirements(algaeArm, elevator);
  }

  @Override
  public void initialize() {
    // lastElevatorPosition = elevator.getElevatorPosition();
  }

  /*@Override
    public void execute() {
      //double currentElevatorPosition = elevator.getElevatorPosition();

      if (currentElevatorPosition > lastElevatorPosition) {
        // Elevator moving up, move AlgaeArm down
        algaeArm.setAlgaeArmPosition(algaeArm.getAlgaeArmPosition() - 0.1);
      } else if (currentElevatorPosition < lastElevatorPosition) {
        // Elevator moving down, move AlgaeArm up
        algaeArm.setAlgaeArmPosition(algaeArm.getAlgaeArmPosition() + 0.1);
      }

      lastElevatorPosition = currentElevatorPosition;
    }
  */
  @Override
  public void end(boolean interrupted) {
    // Optional: Stop the AlgaeArm if needed
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
