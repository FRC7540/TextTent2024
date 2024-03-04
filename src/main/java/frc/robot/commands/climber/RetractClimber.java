package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.util.States.ClimberState;

public class RetractClimber extends Command {
  private final ClimberSubsystem climberSubsystem;
  private boolean endNow = false;

  public RetractClimber(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {
    if (climberSubsystem.getState() == ClimberState.READY
        || climberSubsystem.getState() == ClimberState.EXTENDING
        || climberSubsystem.getState() == ClimberState.RETRACTED
        || climberSubsystem.getState() == ClimberState.UNDEFINED) {
      endNow = true;
      return;
    }
    climberSubsystem.setClimberMotorVoltage(6.0);
  }

  @Override
  public void end(boolean interrupted) {
    climberSubsystem.setClimberMotorVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return endNow
        || climberSubsystem.getState() == ClimberState.RETRACTED
        || climberSubsystem.getState() == ClimberState.UNDEFINED;
  }
}
