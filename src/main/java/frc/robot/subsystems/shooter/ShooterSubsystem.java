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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.util.States.ChamberState;
import frc.robot.util.States.FiringWheelState;
import frc.robot.util.States.FlywheelState;
import frc.robot.util.States.ShooterState;
import java.util.Map;
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

  @AutoLogOutput(key = "Shooter/PusherWheelState")
  private FiringWheelState firingWheelState;

  @AutoLogOutput(key = "Shooter/ChamberState")
  private ChamberState chamberState;

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

  ShuffleboardLayout shooterLayout =
      Shuffleboard.getTab("Teleop")
          .getLayout("Shooter", BuiltInLayouts.kGrid)
          .withPosition(0, 0)
          .withSize(3, 3)
          .withProperties(null);

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
    shooterLayout
        .addDouble("Flywheel 1", () -> flywheelInputs.wheelOneRadSec)
        .withWidget(BuiltInWidgets.kDial)
        .withSize(1, 1)
        .withPosition(0, 0)
        .withProperties(Map.of("min", 0, "max", 40));

    shooterLayout
        .addDouble("Flywheel 2", () -> flywheelInputs.wheelTwoRadSec)
        .withWidget(BuiltInWidgets.kDial)
        .withSize(1, 1)
        .withPosition(1, 0)
        .withProperties(Map.of("min", 0, "max", 40));

    shooterLayout
        .addDouble("Target Speed", () -> targetSpeed)
        .withWidget(BuiltInWidgets.kDial)
        .withSize(1, 1)
        .withPosition(0, 1)
        .withProperties(Map.of("min", 0, "max", 40));

    shooterState = ShooterState.UNDEFINED;
    shooterLayout
        .addString("shooterState", shooterState::toString)
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(1, 1);
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
    if (Robot.isReal()) {
      wheelOneController.latencyCompensate(
          wheelOneFlywheelPlant, Shooter.Flywheel.WheelOne.NOMINAL_DISCRETIZATION_TIMESTEP, 0.002);
      wheelTwoController.latencyCompensate(
          wheelTwoFlywheelPlant, Shooter.Flywheel.WheelTwo.NOMINAL_DISCRETIZATION_TIMESTEP, 0.002);
    }

    wheelOneloop.predict(Shooter.Flywheel.WheelOne.NOMINAL_DISCRETIZATION_TIMESTEP);
    wheelTwoloop.predict(Shooter.Flywheel.WheelTwo.NOMINAL_DISCRETIZATION_TIMESTEP);

    flywheelIO.setWheelOneVoltage(wheelOneloop.getU(0));
    flywheelIO.setWheelTwoVoltage(wheelTwoloop.getU(0));

    determineStates();

    frc.robot.RobotState.shooterState = shooterState;
  }

  private void determineStates() {
    determineChamberState();
    determinePusherWheelStates();
    determineFlywheelState();
    determineShooterState();
  }

  private void determineChamberState() {
    if (getHolderLimitSwitch()) {
      chamberState = ChamberState.LOADED;
      return;
    }
    if (!getHolderLimitSwitch()) {
      chamberState = ChamberState.EMPTY;
      return;
    }
    chamberState = ChamberState.UNDEFINED;
  }

  private void determinePusherWheelStates() {
    if (Math.abs(shooterInputs.firingMotorAppliedVoltage) > 0) {
      firingWheelState = FiringWheelState.FIRING;
      return;
    }

    if (Math.abs(shooterInputs.firingMotorAppliedVoltage) == 0) {
      firingWheelState = FiringWheelState.STOPPED;
      return;
    }
    firingWheelState = FiringWheelState.UNDEFINED;
  }

  private void determineShooterState() {
    if (flywheelState == FlywheelState.STOPPED && chamberState == ChamberState.EMPTY) {
      shooterState = ShooterState.EMPTY;
      return;
    }
    if (flywheelState == FlywheelState.STOPPED && chamberState == ChamberState.LOADED) {
      shooterState = ShooterState.LOADED;
      return;
    }
    if (flywheelState == FlywheelState.SPINNING_UP && chamberState == ChamberState.LOADED) {
      shooterState = ShooterState.ARMING;
      return;
    }
    if (flywheelState == FlywheelState.AT_SPEED && chamberState == ChamberState.LOADED) {
      shooterState = ShooterState.ARMED;
      return;
    }
    if (flywheelState == FlywheelState.AT_SPEED && firingWheelState == FiringWheelState.FIRING) {
      shooterState = ShooterState.SHOOTING;
      return;
    }
    if (flywheelState == FlywheelState.SPINNING_DOWN && chamberState == ChamberState.EMPTY) {
      shooterState = ShooterState.RECOVERING;
      return;
    }
    if (chamberState == ChamberState.EMPTY && firingWheelState == FiringWheelState.FIRING) {
      shooterState = ShooterState.LOADING;
      return;
    }
    if (chamberState == ChamberState.LOADED && flywheelState == FlywheelState.SPINNING_DOWN) {
      shooterState = ShooterState.SAFING;
    }
    shooterState = ShooterState.UNDEFINED;
    return;
  }

  private void determineFlywheelState() {

    double error = Math.abs(targetSpeed - flywheelInputs.wheelOneRadSec);
    error =
        (error > Math.abs(targetSpeed - flywheelInputs.wheelTwoRadSec))
            ? error
            : Math.abs(targetSpeed - flywheelInputs.wheelTwoRadSec);
    boolean errorGreaterThanThreshold = (error > Constants.Shooter.FLYWHEEL_SPEED_THRESHOLD);
    boolean targetSpeedGreaterThanThreshold =
        (Math.abs(targetSpeed) > Constants.Shooter.FLYWHEEL_SPEED_THRESHOLD);

    if ((error <= Constants.Shooter.FLYWHEEL_SPEED_THRESHOLD)
        && (Math.abs(targetSpeed) >= Constants.Shooter.FLYWHEEL_SPEED_THRESHOLD)) {
      flywheelState = FlywheelState.AT_SPEED;
      return;
    }

    if (errorGreaterThanThreshold && targetSpeedGreaterThanThreshold) {
      flywheelState = FlywheelState.SPINNING_UP;
      return;
    }

    if (errorGreaterThanThreshold && !targetSpeedGreaterThanThreshold) {
      flywheelState = FlywheelState.SPINNING_DOWN;
      return;
    }

    if (!errorGreaterThanThreshold && !targetSpeedGreaterThanThreshold) {
      flywheelState = FlywheelState.STOPPED;
      return;
    }

    flywheelState = FlywheelState.UNDEFINED;
  }

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

  public boolean getShotLimitSwitch() {
    return shooterInputs.shotLimitSwitch;
  }

  public boolean getHolderLimitSwitch() {
    return shooterInputs.holdingLimitSwitch;
  }

  /* Returns the necesary shooter flywheel speed to shoot at the current target */
  private double calculateShooterPower(Constants.Targets target) {
    double distanceToTarget = 0.0;
    switch (target) {
      case AMP:
        break;
      case OURSIDE:
        break;
      case SPEAKER:
        distanceToTarget = calculateDistanceToCurrentAliianceSpeaker();
      default:
        break;
    }
    if (!validateShooterTrajectory(distanceToTarget)) {
      return -1.0;
    }
    double noteVelocity = calculateVelocityFromTargetDistance(distanceToTarget);
    double flywheelVelocity = calculateShooterPowerFromTargetVelocity(noteVelocity);

    Logger.recordOutput("Shooter/currentTarget", target);
    Logger.recordOutput("Shooter/distanceToTarget", distanceToTarget);
    Logger.recordOutput("Shooter/targetNoteVelocity", noteVelocity);
    Logger.recordOutput("Shooter/targetFlywheelVelocity", flywheelVelocity);

    return flywheelVelocity;
  }

  private double calculateDistanceToCurrentAliianceSpeaker() {
    return (DriverStation.getAlliance().get() == Alliance.Blue)
        ? RobotState.robotPose2D
            .getTranslation()
            .getDistance(Constants.Field.Blue.SPEAKER_POSE2D.getTranslation())
        : RobotState.robotPose2D
            .getTranslation()
            .getDistance(Constants.Field.Red.SPEAKER_POSE2D.getTranslation());
  }

  private double calculateVelocityFromTargetDistance(double distanceToTarget) {
    // TODO: Implement math
    return 0.0;
  }

  private double calculateShooterPowerFromTargetVelocity(double noteSpeed) {
    // TODO: Implement math
    return 0.0;
  }

  private boolean validateShooterTrajectory(double noteVelocity) {
    return calculateNoteMaximumHeight(noteVelocity) < Constants.Shooter.NOTE_MAXIMUM_HEIGHT_METERS;
  }

  private double calculateNoteMaximumHeight(double noteVelocity) {
    return (Math.pow(noteVelocity, 2)
            * Math.pow(Math.sin(Constants.Shooter.SHOOTER_ANGLE_RADIANS), 2))
        / (2 * Math.abs(-9.80665));
  }

  public ShooterState getState() {
    return shooterState;
  }

  public FlywheelState getFlywheelState() {
    return flywheelState;
  }

  public ChamberState getChamberState() {
    return chamberState;
  }
}
