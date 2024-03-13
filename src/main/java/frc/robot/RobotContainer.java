// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Shooting.FlywheelSpinToTargetVelocity;
import frc.robot.commands.Shooting.ShootNote;
import frc.robot.commands.climber.ExtendClimber;
import frc.robot.commands.climber.RetractClimber;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.DriveWhileLockedToTarget;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOVictor;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import frc.robot.subsystems.drivebase.GyroIO;
import frc.robot.subsystems.drivebase.GyroIONavX;
import frc.robot.subsystems.drivebase.ModuleIO;
import frc.robot.subsystems.drivebase.ModuleIOSim;
import frc.robot.subsystems.drivebase.ModuleIOSparkMax;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelIO;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.FlywheelIOSparkMax;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vison.LimelightIO;
import frc.robot.subsystems.vison.VisionIO;
import frc.robot.subsystems.vison.VisionSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final LoggedDashboardChooser<Command> autoChooser;

  private ShooterSubsystem shooterSubsystem;
  private DrivebaseSubsystem drivebaseSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private VisionSubsystem visionSubsystem;
  private ClimberSubsystem climberSubsystem;

  public CommandXboxController operatorController =
      new CommandXboxController(Constants.HID.operatorControlerPort);
  public CommandXboxController driverController =
      new CommandXboxController(Constants.HID.driverControllerPort);

  // Dashboard bindings
  LoggedDashboardNumber flywheelSpeedInput = new LoggedDashboardNumber("Flywheel speeds");
  LoggedDashboardNumber pathFindX = new LoggedDashboardNumber("pathfinding x");
  LoggedDashboardNumber pathFindY = new LoggedDashboardNumber("pathfinding y");
  LoggedDashboardNumber pathFindTheta = new LoggedDashboardNumber("pathfinding theta");

  PowerDistribution powerDistribution = new PowerDistribution();

  public RobotContainer() {
    if (Robot.isSimulation() && !Robot.isReplay) {
      setupForSimulation();
    } else if (Robot.isReal()) {
      setupForRealRobot();
    }
    fillMissingSubsystems();

    registerVisionConsumers();
    configureDefaultCommands();
    configureBindings();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    DriverStation.silenceJoystickConnectionWarning(true);
    setupDashboard();
  }

  private void setupDashboard() {
    LiveWindow.disableAllTelemetry();
    configureShuffleBoard();
  }

  private void configureShuffleBoard() {

    ShuffleboardLayout warningLightsLayout =
        Shuffleboard.getTab("Teleop")
            .getLayout("Warning Lights", BuiltInLayouts.kGrid)
            .withPosition(0, 5)
            .withSize(5, 2)
            .withProperties(null);

    warningLightsLayout
        .addBoolean("Boolean 1", () -> false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 1)
        .withPosition(0, 0);

    warningLightsLayout
        .addBoolean("Boolean 2", () -> false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 1)
        .withPosition(1, 0);

    warningLightsLayout
        .addBoolean("Boolean 3", () -> false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 2)
        .withPosition(2, 0);

    warningLightsLayout
        .addBoolean("Boolean 4", () -> false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 2)
        .withPosition(3, 0);

    warningLightsLayout
        .addBoolean("Boolean 5", () -> false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 2)
        .withPosition(4, 0);

    Shuffleboard.getTab("Teleop")
        .add("Total Current draw", powerDistribution.getTotalCurrent())
        .withPosition(5, 5)
        .withSize(3, 2)
        .withWidget(BuiltInWidgets.kDial);

    Shuffleboard.getTab("Auto")
        .add("Selected Auto", autoChooser.getSendableChooser())
        .withPosition(0, 0)
        .withSize(2, 2);
  }

  private void configureDefaultCommands() {

    drivebaseSubsystem.setDefaultCommand(
        new DefaultDrive(
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX,
            driverController::getLeftTriggerAxis,
            () -> driverController.rightBumper().debounce(0.2).getAsBoolean(),
            drivebaseSubsystem));
  }

  private void configureBindings() {
    Supplier<Pose2d> currentSpeak =
        () ->
            DriverStation.getAlliance().get() == Alliance.Blue
                ? Constants.Field.Blue.SPEAKER_POSE2D
                : Constants.Field.Red.SPEAKER_POSE2D;

    operatorController
        .a()
        .debounce(0.2)
        .onTrue(new FlywheelSpinToTargetVelocity(shooterSubsystem, () -> flywheelSpeedInput.get()));

    operatorController
        .x()
        .debounce(0.02)
        .onTrue(new frc.robot.commands.IntakeNote(intakeSubsystem, shooterSubsystem));

    operatorController
        .rightTrigger()
        .debounce(0.2)
        .onTrue(new ShootNote(shooterSubsystem, () -> flywheelSpeedInput.get()));

    operatorController
        .rightBumper()
        .debounce(0.2)
        .onTrue(shooterSubsystem.sysIdDynamic(Direction.kForward));
    operatorController
        .leftBumper()
        .debounce(0.2)
        .onTrue(shooterSubsystem.sysIdDynamic(Direction.kReverse));

    operatorController
        .y()
        .debounce(0.2)
        .onTrue(shooterSubsystem.sysIdQuasistatic(Direction.kForward));
    operatorController
        .b()
        .debounce(0.2)
        .onTrue(shooterSubsystem.sysIdQuasistatic(Direction.kReverse));

    driverController.rightBumper().debounce(0.3).whileTrue(new ExtendClimber(climberSubsystem));
    driverController.leftBumper().debounce(0.3).whileTrue(new RetractClimber(climberSubsystem));

    driverController.start().debounce(0.2).onTrue(drivebaseSubsystem.getZeroGyroCommand());
    driverController.x().debounce(0.2).onTrue(drivebaseSubsystem.getZeroPoseCommand());
    driverController
        .a()
        .debounce(1)
        .onTrue(
            AutoBuilder.pathfindToPose(
                new Pose2d(pathFindX.get(), pathFindY.get(), new Rotation2d(pathFindTheta.get())),
                Constants.Drivebase.DEFAULT_PATHFINDING_CONSTRAINTS));

    driverController
        .a()
        .debounce(0.2)
        .onTrue(
            new DriveWhileLockedToTarget(
                currentSpeak,
                driverController::getLeftX,
                driverController::getLeftY,
                driverController::getLeftTriggerAxis,
                drivebaseSubsystem));
  }

  private void registerVisionConsumers() {
    visionSubsystem.registerBotPoseConsumer(drivebaseSubsystem::addVisionMeasurement);
    visionSubsystem.registerBotPoseConsumer(
        (Pose3d pose, Double timestamp) -> {
          RobotState.botVisionPose = pose;
        });
  }

  public void initPathPlanner() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void setupForRealRobot() {
    System.out.println("Real robot detected!");

    // Only create the real IO layer if we need to
    shooterSubsystem = new ShooterSubsystem(new FlywheelIOSparkMax(), new ShooterIOSparkMax() {});

    drivebaseSubsystem =
        new DrivebaseSubsystem(
            new GyroIONavX(),
            new ModuleIOSparkMax(0) {},
            new ModuleIOSparkMax(1) {},
            new ModuleIOSparkMax(2) {},
            new ModuleIOSparkMax(3) {});

    visionSubsystem = new VisionSubsystem(new LimelightIO());

    intakeSubsystem = new IntakeSubsystem(new IntakeIOSparkMax());

    climberSubsystem = new ClimberSubsystem(new ClimberIOVictor() {});
  }

  private void setupForSimulation() {

    System.out.println("Simulation detected! Not set to replay, instantiang simulations.");
    shooterSubsystem = new ShooterSubsystem(new FlywheelIOSim(), new ShooterIOSim());

    drivebaseSubsystem =
        new DrivebaseSubsystem(
            new GyroIO() {},
            new ModuleIOSim() {},
            new ModuleIOSim() {},
            new ModuleIOSim() {},
            new ModuleIOSim() {});
    visionSubsystem = new VisionSubsystem(new VisionIO() {});

    intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());

    climberSubsystem = new ClimberSubsystem(new ClimberIOSim());
  }

  private void fillMissingSubsystems() {
    shooterSubsystem =
        shooterSubsystem != null
            ? shooterSubsystem
            : new ShooterSubsystem(new FlywheelIO() {}, new ShooterIO() {});
    drivebaseSubsystem =
        drivebaseSubsystem != null
            ? drivebaseSubsystem
            : new DrivebaseSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
    visionSubsystem =
        visionSubsystem != null ? visionSubsystem : new VisionSubsystem(new VisionIO() {});
    intakeSubsystem =
        intakeSubsystem != null ? intakeSubsystem : new IntakeSubsystem(new IntakeIO() {});
    climberSubsystem =
        climberSubsystem != null ? climberSubsystem : new ClimberSubsystem(new ClimberIO() {});
  }
}
