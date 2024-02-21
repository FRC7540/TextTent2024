package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeNoteSwitch = false; // limswitch for note enter intake
    public double intakeMotorVoltage = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void driveMotor(double voltage) {}
}
