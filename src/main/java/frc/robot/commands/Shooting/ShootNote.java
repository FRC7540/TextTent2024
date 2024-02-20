package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class ShootNote extends SequentialCommandGroup {

  public ShootNote(ShooterSubsystem shooterSubsystem, DoubleSupplier speed) {
    addCommands(
        new FlywheelSpinToTargetVelocity(shooterSubsystem, speed),
        // Ultimatley we should use actualy feedback from the shooter to wait until the note has
        // been fired
        new ShotPush(shooterSubsystem, Constants.Shooter.Direction.FORWARD)
            .until(shooterSubsystem.getShotLimitSwitch()),
        new FlywheelSpinToTargetVelocity(shooterSubsystem, () -> 0.0));
  }
}
