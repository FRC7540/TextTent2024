package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ShootNote extends SequentialCommandGroup {

  public ShootNote(ShooterSubsystem shooterSubsystem, DoubleSupplier speed) {
    addCommands(
        new FlywheelSpinToTargetVelocity(shooterSubsystem, speed),
        new ShotPush(shooterSubsystem),
        new FlywheelSpinToTargetVelocity(shooterSubsystem, () -> 0.0));
  }
}
