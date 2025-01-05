package frc.robot.simulationSystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ModuleConstants;

public class SwerveModuleSim {
  /** Main timer to simulate the passage of time. */
  private final Timer timer;

  /** Time delta since last update */
  private double dt;

  /** Fake motor position. */
  private double fakePos;

  /**
   * The fake speed of the previous state, used to calculate {@link SwerveModuleSimulation#fakePos}.
   */
  private double fakeSpeed;

  /** Last time queried. */
  private double lastTime;

  /** Current simulated swerve module state. */
  private SwerveModuleState state;

  private DCMotorSim driveMotorSim =
      new DCMotorSim(DCMotor.getNEO(1), ModuleConstants.kDrivingMotorReduction, 0.025);

  public SwerveModuleSim() {
    timer = new Timer();
    timer.start();
    lastTime = timer.get();
    state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    fakeSpeed = 0;
    fakePos = 0;
    dt = 0;
  }

  /**
   * Update the position and state of the module.
   *
   * <p>WARNING: need to be called periodically for simulation to work correctly.
   *
   * @param desiredState State the swerve module should be set to.
   */
  public void updateStateAndPosition(SwerveModuleState desiredState) {
    dt = timer.get() - lastTime;
    lastTime = timer.get();

    state = desiredState;
    fakeSpeed = desiredState.speedMetersPerSecond;
    fakePos += (fakeSpeed * dt);
  }

  /**
   * Get the simulated swerve module position.
   *
   * @return {@link SwerveModulePosition} of the simulated module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(fakePos, state.angle);
  }

  /**
   * Get the {@link SwerveModuleState} of the simulated module.
   *
   * @return {@link SwerveModuleState} of the simulated module.
   */
  public SwerveModuleState getState() {
    return state;
  }
}
