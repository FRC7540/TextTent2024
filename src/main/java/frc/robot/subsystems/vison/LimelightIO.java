package frc.robot.subsystems.vison;

import org.littletonrobotics.junction.AutoLog;

public interface LimelightIO {

  @AutoLog
  public static class LimelightIOInputs {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LimelightIOInputs inputs) {}
}
