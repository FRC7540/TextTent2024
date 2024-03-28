// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.Shooting.FlywheelSpinToTargetVelocity;
import frc.robot.commands.Shooting.ShootNote;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.DriveLockedStickyRotation;
import frc.robot.commands.drive.DriveLockedToNote;
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
import frc.robot.subsystems.noiseCanceling.NoiseCancelingIO;
import frc.robot.subsystems.noiseCanceling.NoiseCancelingIOServo;
import frc.robot.subsystems.noiseCanceling.NoiseCancelingSubsystem;
import frc.robot.subsystems.shooter.FlywheelIO;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.FlywheelIOSparkMax;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vison.AIIO;
import frc.robot.subsystems.vison.AIIOLimelight;
import frc.robot.subsystems.vison.LimelightIO;
import frc.robot.subsystems.vison.VisionIO;
import frc.robot.subsystems.vison.VisionSubsystem;
import frc.robot.util.States.ChamberState;
import frc.robot.util.types.TargetNote;
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
  private NoiseCancelingSubsystem noiseCancelingSubsystem;

  public CommandXboxController operatorController =
      new CommandXboxController(Constants.HID.operatorControlerPort);
  public CommandXboxController driverController =
      new CommandXboxController(Constants.HID.driverControllerPort);

  // Dashboard bindings
  LoggedDashboardNumber flywheelSpeedInput = new LoggedDashboardNumber("Flywheel speeds");
  LoggedDashboardNumber pathFindX = new LoggedDashboardNumber("pathfinding x");
  LoggedDashboardNumber pathFindY = new LoggedDashboardNumber("pathfinding y");
  LoggedDashboardNumber pathFindTheta = new LoggedDashboardNumber("pathfinding theta");

  PowerDistribution powerDistribution =
      new PowerDistribution(51, PowerDistribution.ModuleType.kCTRE);

  public RobotContainer() {
    if (Robot.isSimulation() && !Robot.isReplay) {
      setupForSimulation();
    } else if (Robot.isReal()) {
      setupForRealRobot();
    }
    fillMissingSubsystems();

    registerNamedCommands();
    registerVisionConsumers();
    configureDefaultCommands();
    configureBindings();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    DriverStation.silenceJoystickConnectionWarning(true);
    setupDashboard();
    registerTriggers();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand(
        "ShootNote", new ShootNote(shooterSubsystem, () -> 150.0).withTimeout(2.5));
    NamedCommands.registerCommand("ShootAmp", new ShootNote(shooterSubsystem, () -> 20.0));
    NamedCommands.registerCommand("IntakeNote", new IntakeNote(intakeSubsystem, shooterSubsystem));
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

    HttpCamera AprilTagCamera =
        new HttpCamera("AprilTagCamera", "http://limelight-april.local:5800");
    CameraServer.addCamera(AprilTagCamera);
    Shuffleboard.getTab("Teleop").add(AprilTagCamera).withPosition(11, 0).withSize(9, 4);

    HttpCamera NoteCamera = new HttpCamera("NoteCamera", "http://limelight-ai.local:5800");
    CameraServer.addCamera(NoteCamera);
    Shuffleboard.getTab("Teleop").add(NoteCamera).withPosition(11, 4).withSize(9, 4);

    Shuffleboard.getTab("blah").add("dynfor", drivebaseSubsystem.sysIdDynamic(Direction.kForward));
    Shuffleboard.getTab("blah").add("dynrev", drivebaseSubsystem.sysIdDynamic(Direction.kReverse));

    Shuffleboard.getTab("blah")
        .add("quasfor", drivebaseSubsystem.sysIdQuasistatic(Direction.kForward));
    Shuffleboard.getTab("blah")
        .add("quasrev", drivebaseSubsystem.sysIdQuasistatic(Direction.kReverse));
  }

  private void configureDefaultCommands() {

    drivebaseSubsystem.setDefaultCommand(
        new DefaultDrive(
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX,
            driverController::getLeftTriggerAxis,
            () -> true,
            drivebaseSubsystem));

    // climberSubsystem.setDefaultCommand(
    //     new RunCommand(() -> climberSubsystem.setClimberMotorVoltage(0), climberSubsystem));
  }

  private void configureBindings() {

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
        .debounce(0.02)
        .onTrue(new ShootNote(shooterSubsystem, () -> 80));

    operatorController
        .leftTrigger()
        .debounce(0.02)
        .onTrue(new ShootNote(shooterSubsystem, () -> 150.0));

    operatorController
        .rightBumper()
        .debounce(0.2)
        .onTrue(new ShootNote(shooterSubsystem, () -> 100));

    operatorController.leftBumper().debounce(0.2).onTrue(new ShootNote(shooterSubsystem, () -> 37));

    // driverController.rightBumper().debounce(0.3).whileTrue(new ExtendClimber(climberSubsystem));
    // driverController.leftBumper().debounce(0.3).whileTrue(new RetractClimber(climberSubsystem));
    driverController
        .rightTrigger()
        .debounce(0.4)
        .whileTrue(
            new RunCommand(() -> climberSubsystem.setClimberMotorVoltage(8.0), climberSubsystem));

    driverController
        .start()
        .debounce(0.2)
        .onTrue(
            Commands.sequence(
                drivebaseSubsystem.getZeroGyroCommand(), drivebaseSubsystem.getZeroPoseCommand()));
    // driverController
    //     .a()
    //     .debounce(1)
    //     .onTrue(
    //         AutoBuilder.pathfindToPose(
    //             new Pose2d(pathFindX.get(), pathFindY.get(), new
    // Rotation2d(pathFindTheta.get())),
    //             Constants.Drivebase.DEFAULT_PATHFINDING_CONSTRAINTS));

    driverController
        .a()
        .debounce(0.02)
        .whileTrue(
            new DriveLockedStickyRotation(
                () -> drivebaseSubsystem.getRotation().plus(new Rotation2d(Math.PI)),
                () -> 0.0,
                () -> 0.0,
                () -> 0.0,
                drivebaseSubsystem));

    driverController
        .y()
        .debounce(0.02)
        .whileTrue(
            new DriveLockedStickyRotation(
                drivebaseSubsystem::getRotation,
                driverController::getLeftX,
                driverController::getLeftY,
                driverController::getLeftTriggerAxis,
                drivebaseSubsystem));

    driverController
        .x()
        .debounce(0.02)
        .whileTrue(
            new DriveLockedToNote(
                () -> RobotState.targetNote,
                driverController::getLeftX,
                driverController::getLeftY,
                driverController::getLeftTriggerAxis,
                drivebaseSubsystem));

    driverController
        .leftBumper()
        .debounce(0.02)
        .onTrue(new InstantCommand(() -> drivebaseSubsystem.stopWithX()));
  }

  private void registerVisionConsumers() {
    visionSubsystem.registerBotPoseConsumer(drivebaseSubsystem::addVisionMeasurement);
    visionSubsystem.registerBotPoseConsumer(
        (Pose3d pose, Double timestamp) -> {
          RobotState.botVisionPose = pose;
        });
    visionSubsystem.registerTargetNoteConsumer(
        (TargetNote targetNote, Double timestamp) -> {
          RobotState.targetNote = targetNote;
        });
  }

  public void initPathPlanner() {}

  public void registerTriggers() {
    new Trigger(
            () -> RobotState.targetNote.targetArea() >= Constants.Vision.AUTOINTAKE_AREA_THRESHOLD)
        .and(() -> shooterSubsystem.getChamberState() != ChamberState.EMPTY)
        .debounce(0.2)
        .whileTrue(new IntakeNote(intakeSubsystem, shooterSubsystem));
  }

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
    shooterSubsystem = new ShooterSubsystem(new FlywheelIOSparkMax(), new ShooterIOSparkMax());

    drivebaseSubsystem =
        new DrivebaseSubsystem(
            new GyroIONavX(),
            new ModuleIOSparkMax(0),
            new ModuleIOSparkMax(1),
            new ModuleIOSparkMax(2),
            new ModuleIOSparkMax(3));

    visionSubsystem = new VisionSubsystem(new LimelightIO(), new AIIOLimelight());

    intakeSubsystem = new IntakeSubsystem(new IntakeIOSparkMax());

    climberSubsystem = new ClimberSubsystem(new ClimberIOVictor());

    noiseCancelingSubsystem = new NoiseCancelingSubsystem(new NoiseCancelingIOServo());
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
    visionSubsystem = new VisionSubsystem(new VisionIO() {}, new AIIO() {});

    intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());

    climberSubsystem = new ClimberSubsystem(new ClimberIOSim());

    noiseCancelingSubsystem = new NoiseCancelingSubsystem(new NoiseCancelingIO() {});
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
        visionSubsystem != null
            ? visionSubsystem
            : new VisionSubsystem(new VisionIO() {}, new AIIO() {});

    intakeSubsystem =
        intakeSubsystem != null ? intakeSubsystem : new IntakeSubsystem(new IntakeIO() {});

    climberSubsystem =
        climberSubsystem != null ? climberSubsystem : new ClimberSubsystem(new ClimberIO() {});

    noiseCancelingSubsystem =
        noiseCancelingSubsystem != null
            ? noiseCancelingSubsystem
            : new NoiseCancelingSubsystem(new NoiseCancelingIO() {});
  }
}
