package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.States.IntakeState;

public class IntakeNote extends Command {
  private final IntakeSubsystem inatakeSubsystem;
  private final Timer intakingTimer = new Timer();
  private boolean endNow;

  public IntakeNote(IntakeSubsystem inatakeSubsystem) {
    this.inatakeSubsystem = inatakeSubsystem;
    this.endNow = false;
    addRequirements(inatakeSubsystem);
  }

  @Override
  public void initialize() {
    if (inatakeSubsystem.getState() != IntakeState.EMPTY) {
      this.endNow = true;
    }

    intakingTimer.start();
    inatakeSubsystem.setMotorVoltage(12.0);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    inatakeSubsystem.setMotorVoltage(0.0);
    this.endNow = false;
  }

  @Override
  public boolean isFinished() {
    return inatakeSubsystem.getState() == IntakeState.FULL || this.endNow;
  }
}
