package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.States.FlywheelState;
import java.util.function.DoubleSupplier;

public class FlywheelSpinToTargetVelocity extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final DoubleSupplier goal;

  public FlywheelSpinToTargetVelocity(ShooterSubsystem shooterSubsystem, DoubleSupplier goal) {
    this.shooterSubsystem = shooterSubsystem;
    this.goal = goal;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.setFlywheelSpeeds(goal.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      shooterSubsystem.setFlywheelSpeeds(0.0);
    }
    shooterSubsystem.setFlywheelSpeeds(0.0);
  }

  @Override
  public boolean isFinished() {
    return shooterSubsystem.getFlywheelState() == FlywheelState.AT_SPEED
        || shooterSubsystem.getFlywheelState() == FlywheelState.STOPPED;
  }
}
