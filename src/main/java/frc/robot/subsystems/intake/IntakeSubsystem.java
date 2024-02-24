package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.States.IntakeState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  @AutoLogOutput(key = "Intake/State")
  private IntakeState intakeState;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
    intakeState = IntakeState.EMPTY;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    determineIntakeState();
    frc.robot.RobotState.intakeState = intakeState;
  }

  private void determineIntakeState() {
    if (!getIntakeNoteLimitSwitch() && (getIntakeMotorVoltage() == 0)) {
      intakeState = IntakeState.EMPTY;
      return;
    }

    if (!getIntakeNoteLimitSwitch() && (getIntakeMotorVoltage() > 0)) {
      intakeState = IntakeState.INTAKING;
      return;
    }

    if (getIntakeNoteLimitSwitch() && (getIntakeMotorVoltage() == 0)) {
      intakeState = IntakeState.FULL;
      return;
    }

    if (getIntakeNoteLimitSwitch() && (getIntakeMotorVoltage() > 0)) {
      intakeState = IntakeState.TRANSFERING;
      return;
    }

    if (getIntakeNoteLimitSwitch() && (getIntakeMotorVoltage() < 0)) {
      intakeState = IntakeState.EJECTING;
      return;
    }

    intakeState = IntakeState.UNDEFINED;
  }

  public void setMotorVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public boolean getIntakeNoteLimitSwitch() {
    return inputs.intakeNoteSwitch;
  }

  public double getIntakeMotorVoltage() {
    return inputs.intakeMotorVoltage;
  }

  public IntakeState getState() {
    return intakeState;
  }
}
