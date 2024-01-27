package frc.robot.subsystems.flywheel;

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
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends SubsystemBase {

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  @AutoLogOutput(key = "Flywheel/TargetSpeeds")
  private double targetSpeed = 0.0;

  private final SysIdRoutine sysId;

  private final LinearSystem<N1, N1, N1> wheelOneFlywheelPlant =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getNeo550(Constants.Flywheel.WheelOne.MOTOR_COUNT),
          Constants.Flywheel.WheelOne.MOMENT_OF_INERTIA,
          Constants.Flywheel.WheelOne.GEAR_RATIO);

  private final KalmanFilter<N1, N1, N1> wheelOneObserver =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          wheelOneFlywheelPlant,
          Constants.Flywheel.WheelOne.MODEL_STD_DEV,
          Constants.Flywheel.WheelOne.MEASUREMENT_STD_DEV,
          Constants.Flywheel.WheelOne.NOMINAL_DISCRETIZATION_TIMESTEP);

  private final LinearQuadraticRegulator<N1, N1, N1> wheelOneController =
      new LinearQuadraticRegulator<>(
          wheelOneFlywheelPlant,
          Constants.Flywheel.WheelOne.QELMS,
          Constants.Flywheel.WheelOne.RELMS,
          Constants.Flywheel.WheelOne.NOMINAL_DISCRETIZATION_TIMESTEP);

  private final LinearSystemLoop<N1, N1, N1> wheelOneloop =
      new LinearSystemLoop<>(
          wheelOneFlywheelPlant,
          wheelOneController,
          wheelOneObserver,
          Constants.Flywheel.WheelOne.MAX_VOLTAGE,
          Constants.Flywheel.WheelOne.NOMINAL_DISCRETIZATION_TIMESTEP);

  private final LinearSystem<N1, N1, N1> wheelTwoFlywheelPlant =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getNeo550(Constants.Flywheel.WheelTwo.MOTOR_COUNT),
          Constants.Flywheel.WheelTwo.MOMENT_OF_INERTIA,
          Constants.Flywheel.WheelTwo.GEAR_RATIO);

  private final KalmanFilter<N1, N1, N1> wheelTwoObserver =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          wheelTwoFlywheelPlant,
          Constants.Flywheel.WheelTwo.MODEL_STD_DEV,
          Constants.Flywheel.WheelTwo.MEASUREMENT_STD_DEV,
          Constants.Flywheel.WheelTwo.NOMINAL_DISCRETIZATION_TIMESTEP);

  private final LinearQuadraticRegulator<N1, N1, N1> wheelTwoController =
      new LinearQuadraticRegulator<>(
          wheelTwoFlywheelPlant,
          Constants.Flywheel.WheelTwo.QELMS,
          Constants.Flywheel.WheelTwo.RELMS,
          Constants.Flywheel.WheelTwo.NOMINAL_DISCRETIZATION_TIMESTEP);

  private final LinearSystemLoop<N1, N1, N1> wheelTwoloop =
      new LinearSystemLoop<>(
          wheelTwoFlywheelPlant,
          wheelTwoController,
          wheelTwoObserver,
          Constants.Flywheel.WheelTwo.MAX_VOLTAGE,
          Constants.Flywheel.WheelTwo.NOMINAL_DISCRETIZATION_TIMESTEP);

  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;
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
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (targetSpeed != 0.0) {
      wheelOneloop.setNextR(VecBuilder.fill(targetSpeed));
      wheelTwoloop.setNextR(VecBuilder.fill(targetSpeed));
    } else {
      wheelOneloop.setNextR(VecBuilder.fill(0.0));
      wheelTwoloop.setNextR(VecBuilder.fill(0.0));
    }

    wheelOneloop.correct(VecBuilder.fill(inputs.wheelOneRadSec));
    wheelTwoloop.correct(VecBuilder.fill(inputs.wheelTwoRadSec));
    wheelOneController.latencyCompensate(
        wheelOneFlywheelPlant, Constants.Flywheel.WheelOne.NOMINAL_DISCRETIZATION_TIMESTEP, 0.025);
    wheelTwoController.latencyCompensate(
        wheelTwoFlywheelPlant, Constants.Flywheel.WheelTwo.NOMINAL_DISCRETIZATION_TIMESTEP, 0.025);

    wheelOneloop.predict(Constants.Flywheel.WheelOne.NOMINAL_DISCRETIZATION_TIMESTEP);
    wheelTwoloop.predict(Constants.Flywheel.WheelTwo.NOMINAL_DISCRETIZATION_TIMESTEP);

    io.setWheelOneVoltage(wheelOneloop.getU(0));
    io.setWheelTwoVoltage(wheelTwoloop.getU(0));
  }

  @Override
  public void simulationPeriodic() {}

  public void setBothFlywheelSpeeds(double speed) {
    targetSpeed = speed;
  }

  private void setBothWheelVoltage(double volts) {
    io.setWheelOneVoltage(volts);
    io.setWheelTwoVoltage(volts);
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
    io.setWheelOneVoltage(0.0);
    io.setWheelTwoVoltage(0.0);
  }
}
