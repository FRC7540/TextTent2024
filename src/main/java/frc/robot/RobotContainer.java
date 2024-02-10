// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import frc.robot.subsystems.drivebase.GyroIO;
import frc.robot.subsystems.drivebase.GyroIONavX;
import frc.robot.subsystems.drivebase.ModuleIO;
import frc.robot.subsystems.drivebase.ModuleIOSim;
import frc.robot.subsystems.drivebase.ModuleIOSparkMax;
import frc.robot.subsystems.shooter.FlywheelIO;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.FlywheelIOSparkMax;
import frc.robot.subsystems.shooter.ShooterIO;
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
  private ShooterSubsystem flywheelSubsystem;
  private DrivebaseSubsystem drivebaseSubsystem;
  private VisionSubsystem visionSubsystem;

  public XboxController operatorController =
      new XboxController(Constants.HID.operatorControlerPort);
  public XboxController driverController = new XboxController(Constants.HID.driverControllerPort);

  public RobotContainer() {
    // Instantiate subsystems
    // Simulation
    if (Robot.isSimulation() && !Robot.isReplay) {
      // We are in a simulation, instantiate simulation classes
      System.out.println("Simulation detected!");
      flywheelSubsystem = new ShooterSubsystem(new FlywheelIOSim(), new ShooterIO() {});
      drivebaseSubsystem =
          new DrivebaseSubsystem(
              new GyroIO() {},
              new ModuleIOSim() {},
              new ModuleIOSim() {},
              new ModuleIOSim() {},
              new ModuleIOSim() {});
      visionSubsystem = new VisionSubsystem(new VisionIO() {});

    } else if (Robot.isReal()) {
      // We are on a real robot, instantiate hardware classes
      System.out.println("Real robot detected!");

      // Only create the real IO layer if we need to
      flywheelSubsystem =
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
    }

    // Instantiate missing subsystems

    flywheelSubsystem =
        flywheelSubsystem != null
            ? flywheelSubsystem
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

    registerVisionConsumers();
    configureDefaultCommands();
    configureBindings();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  }

  private void configureBindings() {
    LoggedDashboardNumber moai = new LoggedDashboardNumber("Moai", 0.0);
    Trigger moais = new Trigger(operatorController::getAButton);
    moais.whileTrue(
        new RunCommand(
            () -> {
              flywheelSubsystem.setBothFlywheelSpeeds(moai.get());
            },
            flywheelSubsystem));
  }

  private void configureDefaultCommands() {
    flywheelSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              flywheelSubsystem.setBothFlywheelSpeeds(0);
            },
            flywheelSubsystem));
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
}
