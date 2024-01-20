package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    // Wheel one inputs
    public double wheelOneRadSec = 0.0;
    public double wheelOnePositionRad = 0.0;
    public double wheelOneAppliedVolts = 0.0;
    public double wheelOneAppliedAmps = 0.0;

    // Wheel two inputs
    public double wheelTwoRadSec = 0.0;
    public double wheelTwoPositionRad = 0.0;
    public double wheelTwoAppliedVolts = 0.0;
    public double wheelTwoAppliedAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Sets wheel voltage */
  public default void setWheelOneVoltage(double volts) {}

  /** Sets wheel voltage */
  public default void setWheelTwoVoltage(double volts) {}
}
