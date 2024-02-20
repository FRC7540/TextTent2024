package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double pusherMotorAppliedVoltage = 0.0;
    public boolean holdingLimitSwitch = false;
    public boolean shotLimitSwitch = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setMotorVoltage(double voltage) {}
}
