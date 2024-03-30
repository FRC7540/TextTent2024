package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.States.ShooterState;

public class IntakeNote extends Command {
  private final IntakeSubsystem inatakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final Timer intakingTimer = new Timer();
  private boolean endNow;

  public IntakeNote(IntakeSubsystem inatakeSubsystem, ShooterSubsystem shooterSubsystem) {
    this.inatakeSubsystem = inatakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.endNow = false;
    addRequirements(inatakeSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {
    if (shooterSubsystem.getState() != ShooterState.EMPTY) {
      this.endNow = true;
      return;
    }

    intakingTimer.start();
    inatakeSubsystem.setMotorVoltage(12);
    shooterSubsystem.setPusherVoltage(8);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    inatakeSubsystem.setMotorVoltage(0.0);
    shooterSubsystem.setPusherVoltage(0.0);
    this.endNow = false;
  }

  @Override
  public boolean isFinished() {
    return shooterSubsystem.getState() == ShooterState.LOADED || this.endNow;
  }
}
