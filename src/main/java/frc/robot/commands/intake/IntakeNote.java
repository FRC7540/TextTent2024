package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.States.IntakeState;

public class IntakeNote extends Command {
  private final IntakeSubsystem inatakeSubsystem;
  private final Timer intakingTimer = new Timer();

  public IntakeNote(IntakeSubsystem inatakeSubsystem) {
    this.inatakeSubsystem = inatakeSubsystem;
    addRequirements(inatakeSubsystem);
  }

  @Override
  public void initialize() {
    if (inatakeSubsystem.getState() != IntakeState.EMPTY) {
      this.cancel();
    }

    intakingTimer.start();
    inatakeSubsystem.setMotorVoltage(6.0);
  }

  @Override
  public void execute() {
    if (intakingTimer.hasElapsed(Constants.Intake.INTAKE_TIME_SECONDS)) {
      this.cancel();
    }
  }

  @Override
  public void end(boolean interrupted) {
    inatakeSubsystem.setMotorVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return inatakeSubsystem.getState() == IntakeState.FULL;
  }
}
