// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final LoggedDashboardChooser<Command> autoChooser;
  public final FlywheelSubsystem flywheelSubsystem;

  private final XboxController controller = new XboxController(0);

  public RobotContainer() {
    flywheelSubsystem = new FlywheelSubsystem(new FlywheelIOSparkMax());
    // configureDefaultCommands();
    configureBindings();

    if (Constants.Flags.USE_PATH_PLANNER) {
      autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    } else {
      autoChooser = null;
    }
  }

  private void configureBindings() {
    new JoystickButton(controller, XboxController.Button.kX.value)
        .whileTrue(
            new InstantCommand(
                () -> {
                  flywheelSubsystem.setBothFlywheelSpeeds(500);
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
    return Constants.Flags.USE_PATH_PLANNER ? autoChooser.get() : null;
  }
}
