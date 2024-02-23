package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeNote extends Command {
  private final IntakeSubsystem inatakeSubsystem;
  private final Timer intakingTimer = new Timer();

  public IntakeNote(IntakeSubsystem shooterSubsystem) {
    this.inatakeSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    intakingTimer.start();
    inatakeSubsystem.setMotorVoltage(6.0);
  }

  @Override
  public void end(boolean interrupted) {
    inatakeSubsystem.setMotorVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return intakingTimer.hasElapsed(Constants.Intake.INTAKE_TIME_SECONDS);
  }
}
