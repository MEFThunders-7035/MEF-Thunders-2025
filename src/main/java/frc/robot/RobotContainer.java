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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.simulationSystems.PhotonSim;
import frc.robot.subsystems.AlgaeArmEncoderSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PhotonCameraSystem;
import java.util.Map;
import org.littletonrobotics.urcl.URCL;

public class RobotContainer {
  private final CommandXboxController commandController = new CommandXboxController(0);
  private final XboxController controller = commandController.getHID();

  private final SendableChooser<Command> autoChooser;

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final AlgaeArmEncoderSubsystem algaearmSubsystem = new AlgaeArmEncoderSubsystem();

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
    // add any simulation specific code here.
    PhotonSim.update(driveSubsystem.getPose());
    // was made for photonSim, but it's not used.
  }

  private void setupNamedCommands() {
    NamedCommands.registerCommand("SetX", driveSubsystem.setX());
  }

  private void setDefaultCommands() {
    driveSubsystem.setDefaultCommand(DriveCommands.driveWithController(driveSubsystem, controller));
    ledSubsystem.setDefaultCommand(ledSubsystem.runPattern(LEDPattern.kOff));
    elevatorSubsystem.setDefaultCommand(
        elevatorSubsystem.set(ElevatorSubsystem.ElevatorPosition.IDLE));
  }

  private void configureJoystickBindings() {
    // commandController.a().whileTrue(driveSubsystem.setX());

    commandController.rightBumper().whileTrue(coralSubsystem.takeCoral());

    commandController.leftBumper().whileTrue(coralSubsystem.throwCoral());

    commandController.y().whileTrue(elevatorSubsystem.set(ElevatorSubsystem.ElevatorPosition.L4));
    commandController.y().whileTrue(elevatorSubsystem.set(ElevatorSubsystem.ElevatorPosition.L2));

    // .start is the `start` button on the controller not a `start` function.
    commandController.start().onTrue(driveSubsystem.resetFieldOrientation());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected().andThen(driveSubsystem.stop());
  }
}
