package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.States.IntakeState;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private IntakeState state;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
    state = IntakeState.EMPTY;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (!getIntakeNoteLimitSwitch() && (getIntakeMotorVoltage() == 0)) {
      state = IntakeState.EMPTY;
    } else if (!getIntakeNoteLimitSwitch() && (getIntakeMotorVoltage() > 0)) {
      state = IntakeState.INTAKING;
    } else if (getIntakeNoteLimitSwitch() && (getIntakeMotorVoltage() == 0)) {
      state = IntakeState.FULL;
    } else if (getIntakeNoteLimitSwitch() && (getIntakeMotorVoltage() > 0)) {
      state = IntakeState.TRANSFERING;
    }
  }

  @Override
  public void simulationPeriodic() {}

  public boolean getIntakeNoteLimitSwitch() {
    return inputs.intakeNoteSwitch;
  }

  public double getIntakeMotorVoltage() {
    return inputs.intakeMotorVoltage;
  }
}
