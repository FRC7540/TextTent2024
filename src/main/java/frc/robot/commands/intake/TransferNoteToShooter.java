package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.States.IntakeState;

public class TransferNoteToShooter extends Command {
  private final IntakeSubsystem inatakeSubsystem;

  public TransferNoteToShooter(IntakeSubsystem inatakeSubsystem) {
    this.inatakeSubsystem = inatakeSubsystem;
    addRequirements(inatakeSubsystem);
  }

  @Override
  public void initialize() {
    if (inatakeSubsystem.getState() != IntakeState.FULL) {
      this.cancel();
    }
    inatakeSubsystem.setMotorVoltage(6.0);
  }

  @Override
  public void end(boolean interrupted) {
    inatakeSubsystem.setMotorVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return inatakeSubsystem.getState() != IntakeState.TRANSFERING;
  }
}
