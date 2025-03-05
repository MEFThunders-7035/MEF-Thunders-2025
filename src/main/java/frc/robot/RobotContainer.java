package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PlaceReefCommands;
import frc.robot.simulationSystems.PhotonSim;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

import java.util.Map;
import org.littletonrobotics.urcl.URCL;

public class RobotContainer {
  private final CommandXboxController commandController = new CommandXboxController(0);
  private final XboxController controller = commandController.getHID();

  private final SendableChooser<Command> autoChooser;

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  public RobotContainer() {
    setupNamedCommands();
    loggingInit();
    configureJoystickBindings();
    setDefaultCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    PhotonCameraSystem.getAprilTagWithID(0); // Load the class before enable.
    SmartDashboard.putData("Auto Chooser", autoChooser);
    setupCamera();
  }

  private void setupCamera() {
    CameraServer.startAutomaticCapture();
  }

  private void loggingInit() {
    DataLogManager.start();
    URCL.start(Map.of());
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  public void simPeriodic() {
    PhotonSim.update(driveSubsystem.getPose());
  }

  private void setupNamedCommands() {
    NamedCommands.registerCommand("SetX", driveSubsystem.setX());
  }

  private void setDefaultCommands() {
    driveSubsystem.setDefaultCommand(DriveCommands.driveWithController(driveSubsystem, controller));
    ledSubsystem.setDefaultCommand(ledSubsystem.runPattern(LEDPattern.kOff));
    elevatorSubsystem.setDefaultCommand(
        elevatorSubsystem.set(ElevatorPosition.IDLE));
    armSubsystem.setDefaultCommand(armSubsystem.set(ArmSubsystem.ArmPosition.IDLE));
  }

  private void configureJoystickBindings() {
    commandController
        .rightBumper()
        .whileTrue(
            coralSubsystem
                .takeCoral()
                .alongWith(elevatorSubsystem.set(ElevatorSubsystem.ElevatorPosition.INTAKE)));
    commandController.leftBumper().whileTrue(coralSubsystem.throwCoral());
    commandController.back().whileTrue(coralSubsystem.moveBackwardForHelp());
    commandController
        .y()
        .whileTrue(PlaceReefCommands.L2(elevatorSubsystem, armSubsystem));
    commandController
        .b()
        .whileTrue(PlaceReefCommands.L3(elevatorSubsystem, armSubsystem));
    
    commandController.x().whileTrue(
      Commands.parallel(
      armSubsystem.set(ArmPosition.ALGEE),
      elevatorSubsystem.set(ElevatorPosition.ALGEE_L1)
    ));
    commandController.start().onTrue(driveSubsystem.resetFieldOrientation());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected().andThen(driveSubsystem.drive(0, 0, 0));
  }
}
