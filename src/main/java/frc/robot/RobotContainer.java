package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.simulationSystems.PhotonSim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PhotonCameraSystem;
import java.util.Map;
import org.littletonrobotics.urcl.URCL;

public class RobotContainer {
  private final CommandXboxController commandController = new CommandXboxController(0);
  private final XboxController controller = commandController.getHID();

  private final SendableChooser<Command> autoChooser;

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  public RobotContainer() {
    setupNamedCommands();
    loggingInit();
    configureJoystickBindings();
    setDefaultCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    PhotonCameraSystem.getAprilTagWithID(0); // Load the class before enable.
    SmartDashboard.putData("Auto Chooser", autoChooser);
    if (RobotBase.isSimulation()) {
      simInit();
    }
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

  private Thread simThread;

  public void simInit() {
    simThread =
        new Thread(
            () -> {
              System.out.println("Starting PhotonSim");
              while (true) {
                PhotonSim.update(driveSubsystem.getPose());
                // I do not want to use a busy loop, so I added a delay.
                Timer.delay(0.05);
              }
            },
            "simThread");
    simThread.setDaemon(true);
    simThread.start();
  }

  public void simPeriodic() {
    if (!simThread.isAlive()) {
      simInit(); // If the thread dies, restart it.
      // This is here because sometimes the thread throws an exception and dies.
    }
    // add any simulation specific code here.
    // was made for photonSim, but it's not used.
  }

  private void setupNamedCommands() {
    NamedCommands.registerCommand("SetX", driveSubsystem.setX());
  }

  private void setDefaultCommands() {
    driveSubsystem.setDefaultCommand(DriveCommands.driveWithController(driveSubsystem, controller));
    ledSubsystem.setDefaultCommand(ledSubsystem.runPattern(LEDPattern.kOff));
  }

  private void configureJoystickBindings() {
    commandController.a().whileTrue(driveSubsystem.setX());

    // .start is the `start` button on the controller not a `start` function.
    commandController.start().onTrue(driveSubsystem.resetFieldOrientation());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected().andThen(driveSubsystem.stop());
  }
}
