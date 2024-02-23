// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.intake.IntakeNote;
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
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vison.LimelightIO;
import frc.robot.subsystems.vison.VisionIO;
import frc.robot.subsystems.vison.VisionSubsystem;
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

  public CommandXboxController operatorController =
      new CommandXboxController(Constants.HID.operatorControlerPort);
  public CommandXboxController driverController =
      new CommandXboxController(Constants.HID.driverControllerPort);

  // Dashboard bindings
  LoggedDashboardNumber flywheelSpeedInput = new LoggedDashboardNumber("Flywheel speeds");
  LoggedDashboardNumber pathFindX = new LoggedDashboardNumber("pathfinding x");
  LoggedDashboardNumber pathFindY = new LoggedDashboardNumber("pathfinding y");
  LoggedDashboardNumber pathFindTheta = new LoggedDashboardNumber("pathfinding theta");

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
  }

  private void configureBindings() {
    operatorController
        .a()
        .debounce(0.2)
        .onTrue(
            new InstantCommand(() -> shooterSubsystem.setFlywheelSpeeds(flywheelSpeedInput.get())));

    operatorController
        .b()
        .debounce(0.2)
        .onTrue(
            new InstantCommand(() -> shooterSubsystem.setFlywheelSpeeds(flywheelSpeedInput.get())));

    operatorController.x().debounce(0.02).onTrue(new IntakeNote(intakeSubsystem));

    driverController.start().debounce(0.2).onTrue(drivebaseSubsystem.getZeroGyroCommand());
    driverController.x().debounce(0.2).onTrue(drivebaseSubsystem.getZeroPoseCommand());
    driverController
        .a()
        .debounce(1)
        .onTrue(
            AutoBuilder.pathfindToPose(
                new Pose2d(pathFindX.get(), pathFindY.get(), new Rotation2d(pathFindTheta.get())),
                Constants.Drivebase.DEFAULT_PATHFINDING_CONSTRAINTS));
  }

  private void configureDefaultCommands() {
    drivebaseSubsystem.setDefaultCommand(
        new DefaultDrive(
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX,
            driverController::getLeftTriggerAxis,
            drivebaseSubsystem));
  }

  private void registerVisionConsumers() {
    visionSubsystem.registerBotPoseConsumer(drivebaseSubsystem::addVisionMeasurement);
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
    shooterSubsystem =
        Preferences.getBoolean("flywheelReal", Constants.Flags.USE_REAL_FLYWHEEL_HARDWARE)
            ? new ShooterSubsystem(new FlywheelIOSparkMax(), new ShooterIO() {})
            : new ShooterSubsystem(new FlywheelIO() {}, new ShooterIO() {});

    drivebaseSubsystem =
        new DrivebaseSubsystem(
            new GyroIONavX(),
            new ModuleIOSparkMax(0) {},
            new ModuleIOSparkMax(1) {},
            new ModuleIOSparkMax(2) {},
            new ModuleIOSparkMax(3) {});

    visionSubsystem = new VisionSubsystem(new LimelightIO());

    intakeSubsystem = new IntakeSubsystem(new IntakeIOSparkMax());
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
  }
}
