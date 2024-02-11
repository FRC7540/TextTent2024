package frc.robot.subsystems.shooter;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Shooter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private final FlywheelIO flywheelIO;
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  @AutoLogOutput(key = "Flywheel/TargetSpeeds")
  private double targetSpeed = 0.0;

  private final SysIdRoutine sysId;

  private final LinearSystem<N1, N1, N1> wheelOneFlywheelPlant =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getNeo550(Shooter.Flywheel.WheelOne.MOTOR_COUNT),
          Shooter.Flywheel.WheelOne.MOMENT_OF_INERTIA,
          Shooter.Flywheel.WheelOne.GEAR_RATIO);

  private final KalmanFilter<N1, N1, N1> wheelOneObserver =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          wheelOneFlywheelPlant,
          Shooter.Flywheel.WheelOne.MODEL_STD_DEV,
          Shooter.Flywheel.WheelOne.MEASUREMENT_STD_DEV,
          Shooter.Flywheel.WheelOne.NOMINAL_DISCRETIZATION_TIMESTEP);

  private final LinearQuadraticRegulator<N1, N1, N1> wheelOneController =
      new LinearQuadraticRegulator<>(
          wheelOneFlywheelPlant,
          Shooter.Flywheel.WheelOne.QELMS,
          Shooter.Flywheel.WheelOne.RELMS,
          Shooter.Flywheel.WheelOne.NOMINAL_DISCRETIZATION_TIMESTEP);

  private final LinearSystemLoop<N1, N1, N1> wheelOneloop =
      new LinearSystemLoop<>(
          wheelOneFlywheelPlant,
          wheelOneController,
          wheelOneObserver,
          Shooter.Flywheel.WheelOne.MAX_VOLTAGE,
          Shooter.Flywheel.WheelOne.NOMINAL_DISCRETIZATION_TIMESTEP);

  private final LinearSystem<N1, N1, N1> wheelTwoFlywheelPlant =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getNeo550(Shooter.Flywheel.WheelTwo.MOTOR_COUNT),
          Shooter.Flywheel.WheelTwo.MOMENT_OF_INERTIA,
          Shooter.Flywheel.WheelTwo.GEAR_RATIO);

  private final KalmanFilter<N1, N1, N1> wheelTwoObserver =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          wheelTwoFlywheelPlant,
          Shooter.Flywheel.WheelTwo.MODEL_STD_DEV,
          Shooter.Flywheel.WheelTwo.MEASUREMENT_STD_DEV,
          Shooter.Flywheel.WheelTwo.NOMINAL_DISCRETIZATION_TIMESTEP);

  private final LinearQuadraticRegulator<N1, N1, N1> wheelTwoController =
      new LinearQuadraticRegulator<>(
          wheelTwoFlywheelPlant,
          Shooter.Flywheel.WheelTwo.QELMS,
          Shooter.Flywheel.WheelTwo.RELMS,
          Shooter.Flywheel.WheelTwo.NOMINAL_DISCRETIZATION_TIMESTEP);

  private final LinearSystemLoop<N1, N1, N1> wheelTwoloop =
      new LinearSystemLoop<>(
          wheelTwoFlywheelPlant,
          wheelTwoController,
          wheelTwoObserver,
          Shooter.Flywheel.WheelTwo.MAX_VOLTAGE,
          Shooter.Flywheel.WheelTwo.NOMINAL_DISCRETIZATION_TIMESTEP);

  public ShooterSubsystem(FlywheelIO flywheelIO, ShooterIO shooterIO) {
    this.flywheelIO = flywheelIO;
    this.shooterIO = shooterIO;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  setBothWheelVoltage(voltage.in(Units.Volts));
                },
                null,
                this));
  }

  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter/Main", flywheelInputs);

    if (targetSpeed != 0.0) {
      wheelOneloop.setNextR(VecBuilder.fill(targetSpeed));
      wheelTwoloop.setNextR(VecBuilder.fill(targetSpeed));
    } else {
      wheelOneloop.setNextR(VecBuilder.fill(0.0));
      wheelTwoloop.setNextR(VecBuilder.fill(0.0));
    }

    wheelOneloop.correct(VecBuilder.fill(flywheelInputs.wheelOneRadSec));
    wheelTwoloop.correct(VecBuilder.fill(flywheelInputs.wheelTwoRadSec));
    wheelOneController.latencyCompensate(
        wheelOneFlywheelPlant, Shooter.Flywheel.WheelOne.NOMINAL_DISCRETIZATION_TIMESTEP, 0.025);
    wheelTwoController.latencyCompensate(
        wheelTwoFlywheelPlant, Shooter.Flywheel.WheelTwo.NOMINAL_DISCRETIZATION_TIMESTEP, 0.025);

    wheelOneloop.predict(Shooter.Flywheel.WheelOne.NOMINAL_DISCRETIZATION_TIMESTEP);
    wheelTwoloop.predict(Shooter.Flywheel.WheelTwo.NOMINAL_DISCRETIZATION_TIMESTEP);

    flywheelIO.setWheelOneVoltage(wheelOneloop.getU(0));
    flywheelIO.setWheelTwoVoltage(wheelTwoloop.getU(0));
  }

  @Override
  public void simulationPeriodic() {}

  public void setBothFlywheelSpeeds(double speed) {
    targetSpeed = speed;
  }

  private void setBothWheelVoltage(double volts) {
    flywheelIO.setWheelOneVoltage(volts);
    flywheelIO.setWheelTwoVoltage(volts);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public void stop() {
    wheelOneloop.setNextR(VecBuilder.fill(0.0));
    wheelTwoloop.setNextR(VecBuilder.fill(0.0));
    flywheelIO.setWheelOneVoltage(0.0);
    flywheelIO.setWheelTwoVoltage(0.0);
  }
}