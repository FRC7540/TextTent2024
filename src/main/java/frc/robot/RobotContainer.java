// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
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
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.flywheel.ShooterIO;
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
  public final FlywheelSubsystem flywheelSubsystem;
  public final DrivebaseSubsystem drivebaseSubsystem;

  public XboxController operatorController =
      new XboxController(Constants.HID.operatorControlerPort);
  public XboxController driverController = new XboxController(Constants.HID.driverControllerPort);

  public RobotContainer() {
    // Instantiate subsystems
    // Simulation
    if (Robot.isSimulation() && !Robot.isReplay) {
      // We are in a simulation, instantiate simulation classes
      System.out.println("Simulation detected! Not set to replay, instantiang simulations.");
      flywheelSubsystem = new FlywheelSubsystem(new FlywheelIOSim(), new ShooterIO() {});
      drivebaseSubsystem =
          new DrivebaseSubsystem(
              new GyroIO() {},
              new ModuleIOSim() {},
              new ModuleIOSim() {},
              new ModuleIOSim() {},
              new ModuleIOSim() {});
    } else if (Robot.isReal()) {
      // We are on a real robot, instantiate hardware classes
      System.out.println("Real robot detected! Instantiating subsystems.");
      flywheelSubsystem = new FlywheelSubsystem(new FlywheelIOSparkMax(), new ShooterIO() {});
      drivebaseSubsystem =
          new DrivebaseSubsystem(
              new GyroIONavX(),
              new ModuleIOSparkMax(0) {},
              new ModuleIOSparkMax(1) {},
              new ModuleIOSparkMax(2) {},
              new ModuleIOSparkMax(3) {});
    } else {
      // Fill evrythign else, we are probally in a replay!
      System.out.println("We must be in a replay! Instantiating bare I/O layers.");
      flywheelSubsystem = new FlywheelSubsystem(new FlywheelIO() {}, new ShooterIO() {});

      drivebaseSubsystem =
          new DrivebaseSubsystem(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }

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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
