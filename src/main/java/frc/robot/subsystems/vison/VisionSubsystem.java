package frc.robot.subsystems.vison;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.VirtualSubsystem;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends VirtualSubsystem {

  private final VisionIO limelightIO;
  private final VisionIOInputsAutoLogged limelightInputs = new VisionIOInputsAutoLogged();

  private ArrayList<PoseConsumer> botPoseConsumers = new ArrayList<PoseConsumer>();
  private ArrayList<PoseConsumer> targetConsumers = new ArrayList<PoseConsumer>();

  public VisionSubsystem(VisionIO visionio) {
    this.limelightIO = visionio;
  }

  @Override
  public void periodic() {
    limelightIO.updateInputs(limelightInputs);
    Logger.processInputs("Vision/Limelight", limelightInputs);

    for (PoseConsumer poseConsumer : botPoseConsumers) {
      poseConsumer.accept(limelightInputs.selfPoseFieldSpace, limelightInputs.captureTimestamp);
    }

    for (PoseConsumer poseConsumer : targetConsumers) {
      poseConsumer.accept(limelightInputs.targetPoseRobotSpace, limelightInputs.captureTimestamp);
    }
  }

  @FunctionalInterface
  public interface PoseConsumer {
    void accept(Pose3d pose, double timestamp);
  }

  public void registerBotPoseConsumer(PoseConsumer poseConsumer) {
    botPoseConsumers.add(poseConsumer);
  }

  public void registerTargerPoseConsumer(PoseConsumer poseConsumer) {
    targetConsumers.add(poseConsumer);
  }

  public void removeBotPoseConsumer(PoseConsumer poseConsumer) {
    botPoseConsumers.remove(poseConsumer);
  }

  public void removeTargerPoseConsumer(PoseConsumer poseConsumer) {
    targetConsumers.remove(poseConsumer);
  }
}
