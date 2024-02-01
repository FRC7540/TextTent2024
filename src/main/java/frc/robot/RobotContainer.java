// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import frc.robot.subsystems.drivebase.GyroIO;
import frc.robot.subsystems.drivebase.GyroIONavX;
import frc.robot.subsystems.drivebase.ModuleIOSim;
import frc.robot.subsystems.drivebase.ModuleIOSparkMax;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
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

  public RobotContainer() {
    if (Robot.isSimulation()) {
      flywheelSubsystem = new FlywheelSubsystem(new FlywheelIOSim());
      drivebaseSubsystem =
          new DrivebaseSubsystem(
              new GyroIO() {},
              new ModuleIOSim() {},
              new ModuleIOSim() {},
              new ModuleIOSim() {},
              new ModuleIOSim() {});
    } else {
      flywheelSubsystem = new FlywheelSubsystem(new FlywheelIOSparkMax());
      drivebaseSubsystem =
          new DrivebaseSubsystem(
              new GyroIONavX(),
              new ModuleIOSparkMax(0) {},
              new ModuleIOSparkMax(1) {},
              new ModuleIOSparkMax(2) {},
              new ModuleIOSparkMax(3) {});
    }

    // configureDefaultCommands();
    configureBindings();

    if (Preferences.getBoolean("USEPATHPLANNER", false)) {
      autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    } else {
      autoChooser = null;
    }
  }

  private void configureBindings() {
    LoggedDashboardNumber moai = new LoggedDashboardNumber("Moai", 0.0);
    SmartDashboard.putData(
        new RunCommand(
            () -> {
              flywheelSubsystem.setBothFlywheelSpeeds(moai.get());
            },
            flywheelSubsystem));
  }

  private void configureDefaultCommands() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Preferences.getBoolean("USEPATHPLANNER", false) ? autoChooser.get() : null;
  }
}
