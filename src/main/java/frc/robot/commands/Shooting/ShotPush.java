package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShotPush extends Command {
  private final ShooterSubsystem shooterSubsystem;

  public ShotPush(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.setPusherVoltage(12.0);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setPusherVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    Trigger trig = new Trigger(shooterSubsystem::getShotLimitSwitch);
    return trig.debounce(0.1).getAsBoolean();
  }
}
