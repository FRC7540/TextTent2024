package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.States.ShooterState;

public class ShotPush extends Command {
  private final ShooterSubsystem shooterSubsystem;

  public ShotPush(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    if (shooterSubsystem.getState() != ShooterState.ARMED) {
      this.cancel();
    }
    shooterSubsystem.setPusherVoltage(Constants.Shooter.Direction.FORWARD.getVoltage());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setPusherVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return shooterSubsystem.getState() == ShooterState.RECOVERING;
  }
}
