package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.States.ShooterState;

public class IntakeToShooterFromIntake extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private boolean cancelNow;

  public Timer transferingTimer = new Timer();

  public IntakeToShooterFromIntake(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.cancelNow = false;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    if (shooterSubsystem.getState() != ShooterState.EMPTY
        || shooterSubsystem.getState() != ShooterState.RECOVERING) {
      this.cancelNow = true;
    }
    shooterSubsystem.setPusherVoltage(Constants.Shooter.Direction.FORWARD.getVoltage());
    transferingTimer.start();
  }

  @Override
  public void execute() {
    if (transferingTimer.hasElapsed(5)) {
      this.cancelNow = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setPusherVoltage(Constants.Shooter.Direction.STOP.getVoltage());
    this.cancelNow = false;
  }

  @Override
  public boolean isFinished() {
    return shooterSubsystem.getState() == ShooterState.LOADED || this.cancelNow == true;
  }
}