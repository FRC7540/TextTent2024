package frc.robot.subsystems.vison;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public double captureTimestamp = 0.0;
    public int currentPipeline = -1;
    public boolean validEntry = false;
    public Pose3d selfPoseFieldSpace = new Pose3d(new Translation3d(), new Rotation3d());
    public Pose3d targetPoseRobotSpace = new Pose3d(new Translation3d(), new Rotation3d());
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Sets the device to use this pipeline */
  public default void setPipeline(int pipeline) {}
}
