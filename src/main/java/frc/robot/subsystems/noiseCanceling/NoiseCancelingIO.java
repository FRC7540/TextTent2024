package frc.robot.subsystems.noiseCanceling;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface NoiseCancelingIO {

  @AutoLog
  public static class NoiseCancelingInputs {
    public Rotation2d servoOneRotation = new Rotation2d();
    public Rotation2d servoTwoRotation = new Rotation2d();
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(NoiseCancelingInputs inputs) {}

  public default void setMotorOne(Rotation2d rotation) {}

  public default void setMotorTwo(Rotation2d rotation) {}
}
