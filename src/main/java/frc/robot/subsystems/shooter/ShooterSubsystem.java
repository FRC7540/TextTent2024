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
import frc.robot.util.States.FlywheelState;
import frc.robot.util.States.ShooterState;
import java.util.function.BooleanSupplier;
import java.util.function.DoublePredicate;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private final FlywheelIO flywheelIO;
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  @AutoLogOutput(key = "Shooter/FlywheelState")
  private FlywheelState flywheelState;

  @AutoLogOutput(key = "Shooter/ShooterState")
  private ShooterState shooterState;

  public static double optimalRotationSpeed = 100;

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
    Logger.processInputs("Shooter/Main", shooterInputs);

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

    // State

    flywheelState = FlywheelState.STOPPED;

    if (((-0.5 <= flywheelInputs.wheelOneRadSec) && (flywheelInputs.wheelOneRadSec <= 0.5))
        && ((-0.5 <= flywheelInputs.wheelTwoRadSec) && (flywheelInputs.wheelOneRadSec <= 0.5))) {
      flywheelState = FlywheelState.STOPPED;
    } else if (flywheelInputs.wheelOneRadSec <= targetSpeed
        && flywheelInputs.wheelTwoRadSec <= targetSpeed) {
      flywheelState = FlywheelState.SPINNING_UP;
    } else if ((((targetSpeed - 3) <= flywheelInputs.wheelOneRadSec)
            && (flywheelInputs.wheelOneRadSec <= (targetSpeed + 3)))
        && (((targetSpeed - 3) <= flywheelInputs.wheelTwoRadSec)
            && (flywheelInputs.wheelTwoRadSec <= (targetSpeed + 3)))) {
      flywheelState = FlywheelState.AT_SPEED;
    } else if (flywheelInputs.wheelOneRadSec >= targetSpeed
        && flywheelInputs.wheelTwoRadSec >= targetSpeed) {
      flywheelState = FlywheelState.SPINNING_DOWN;
    }

    getHolderLimitSwitch().getAsBoolean();
    getShotLimitSwitch().getAsBoolean();

    if (flywheelState == FlywheelState.STOPPED && getHolderLimitSwitch().getAsBoolean() == false) {
      shooterState = ShooterState.EMPTY;
    } else if (flywheelState == FlywheelState.STOPPED
        && getHolderLimitSwitch().getAsBoolean() == true) {
      shooterState = ShooterState.LOADED;
    } else if (flywheelState == FlywheelState.SPINNING_UP
        && getHolderLimitSwitch().getAsBoolean() == true) {
      shooterState = ShooterState.ARMING;
    } else if (flywheelState == FlywheelState.AT_SPEED
        && getHolderLimitSwitch().getAsBoolean() == true) {
      shooterState = ShooterState.ARMED;
    } else if (flywheelState == FlywheelState.AT_SPEED
        && getShotLimitSwitch().getAsBoolean() == true) {
      shooterState = ShooterState.SHOOTING;
    } else if (flywheelState == FlywheelState.SPINNING_DOWN
        && getHolderLimitSwitch().getAsBoolean() == false) {
      shooterState = ShooterState.RECOVERING;
    }
  }

  @Override
  public void simulationPeriodic() {}

  public void setFlywheelSpeeds(double speed) {
    targetSpeed = speed;
  }

  private void setBothWheelVoltage(double volts) {
    flywheelIO.setWheelOneVoltage(volts);
    flywheelIO.setWheelTwoVoltage(volts);
  }

  public boolean flywheelCompareVelocity(DoublePredicate velocity) {
    return velocity.test(flywheelInputs.wheelOneRadSec)
        && velocity.test(flywheelInputs.wheelTwoRadSec);
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

  public void setPusherVoltage(double voltage) {
    shooterIO.setMotorVoltage(voltage);
  }

  public BooleanSupplier getShotLimitSwitch() {
    return () -> shooterInputs.shotLimitSwitch;
  }

  public BooleanSupplier getHolderLimitSwitch() {
    return () -> shooterInputs.holdingLimitSwitch;
  }
}
