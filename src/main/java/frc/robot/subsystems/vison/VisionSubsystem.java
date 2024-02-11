package frc.robot.subsystems.vison;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.VirtualSubsystem;
import java.util.ArrayList;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends VirtualSubsystem {

  private final VisionIO limelightIO;
  private final VisionIOInputsAutoLogged limelightInputs = new VisionIOInputsAutoLogged();
  private double lastTimestamp = 0;

  private ArrayList<BiConsumer<Pose3d, Double>> botPoseConsumers =
      new ArrayList<BiConsumer<Pose3d, Double>>();
  private ArrayList<BiConsumer<Pose3d, Double>> targetConsumers =
      new ArrayList<BiConsumer<Pose3d, Double>>();

  public VisionSubsystem(VisionIO visionio) {
    this.limelightIO = visionio;
  }

  @Override
  public void periodic() {
    limelightIO.updateInputs(limelightInputs);
    Logger.processInputs("Vision/Limelight", limelightInputs);

    if (limelightInputs.captureTimestamp != lastTimestamp) {
      lastTimestamp = limelightInputs.captureTimestamp;
      for (BiConsumer<Pose3d, Double> botPoseConsumer : botPoseConsumers) {
        botPoseConsumer.accept(
            limelightInputs.selfPoseFieldSpace, limelightInputs.captureTimestamp);
      }

      for (BiConsumer<Pose3d, Double> targetConsumer : targetConsumers) {
        targetConsumer.accept(
            limelightInputs.targetPoseRobotSpace, limelightInputs.captureTimestamp);
      }
    }
  }

  public void registerBotPoseConsumer(BiConsumer<Pose3d, Double> poseConsumer) {
    botPoseConsumers.add(poseConsumer);
  }

  public void registerTargerPoseConsumer(BiConsumer<Pose3d, Double> poseConsumer) {
    targetConsumers.add(poseConsumer);
  }

  public void removeBotPoseConsumer(BiConsumer<Pose3d, Double> poseConsumer) {
    botPoseConsumers.remove(poseConsumer);
  }

  public void removeTargerPoseConsumer(BiConsumer<Pose3d, Double> poseConsumer) {
    targetConsumers.remove(poseConsumer);
  }
}
