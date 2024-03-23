package frc.robot.subsystems.vison;

import org.littletonrobotics.junction.AutoLog;

public interface AIIO {

  @AutoLog
  public static class AIInputs {
    public double captureTimestamp = 0.0;
    public int currentPipeline = -1;
    public boolean validEntry = false;
    public double xError = 0.0;
    public double yError = 0.0;
    public double targetArea = 0.0;
    public String targetClass = "";
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(AIInputs inputs) {}

  /** Sets the device to use this pipeline */
  public default void setPipeline(int pipeline) {}
}
