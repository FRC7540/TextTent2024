package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShotPush extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private double speed;

  public ShotPush(ShooterSubsystem shooterSubsystem, double speed) {
    this.shooterSubsystem = shooterSubsystem;
    this.speed = speed;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.setFlywheelSpeeds(speed);
    shooterSubsystem.setPusherVoltage(12.0);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setPusherVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    Trigger trig = new Trigger(shooterSubsystem::getShotLimitSwitch);
    return false;
  }
}
