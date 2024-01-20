package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    // Wheel one inputs
    public double wheelOneRadSec = 0.0;

    // Wheel two inputs
    public double wheelTwoRadSec = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Sets wheel voltage */
  public default void setWheelOneVoltage(double volts) {}

  /** Sets wheel voltage */
  public default void setWheelTwoVoltage(double volts) {}
}
