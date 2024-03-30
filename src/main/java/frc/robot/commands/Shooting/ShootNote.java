package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ShootNote extends SequentialCommandGroup {

  public ShootNote(ShooterSubsystem shooterSubsystem, DoubleSupplier speed) {
    addCommands(
        new FlywheelSpinToTargetVelocity(shooterSubsystem, speed).withTimeout(1.0),
        // new WaitCommand(0.5),
        new ShotPush(shooterSubsystem, speed.getAsDouble()).withTimeout(1.5),
        new FlywheelSpinToTargetVelocity(shooterSubsystem, () -> 0.0).withTimeout(1));
  }
}
