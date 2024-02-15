package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShotPush extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final Constants.Shooter.Direction direction;

  public ShotPush(ShooterSubsystem shooterSubsystem, Constants.Shooter.Direction direction) {
    this.shooterSubsystem = shooterSubsystem;
    this.direction = direction;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.setPusherVoltage(direction.getVoltage());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setPusherVoltage(0.0);
  }
}
