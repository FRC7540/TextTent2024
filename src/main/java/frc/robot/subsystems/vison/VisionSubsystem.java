package frc.robot.subsystems.vison;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.types.TargetNote;
import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.IntSupplier;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends VirtualSubsystem {

  private final VisionIO limelightIO;
  private final VisionIOInputsAutoLogged limelightInputs = new VisionIOInputsAutoLogged();
  private double lastTimestampApril = 0;

  private final AIIO aiio;
  private final AIInputsAutoLogged aiInputs = new AIInputsAutoLogged();
  private double lastTimestampAi = 0;

  private ArrayList<BiConsumer<Pose3d, Double>> botPoseConsumers =
      new ArrayList<BiConsumer<Pose3d, Double>>();
  private ArrayList<BiConsumer<Pose3d, Double>> targetConsumers =
      new ArrayList<BiConsumer<Pose3d, Double>>();

  private ArrayList<BiConsumer<TargetNote, Double>> targetNoteconsumers =
      new ArrayList<BiConsumer<TargetNote, Double>>();

  public VisionSubsystem(VisionIO visionio, AIIO aiio) {
    this.limelightIO = visionio;
    this.aiio = aiio;
  }

  @Override
  public void periodic() {
    limelightIO.updateInputs(limelightInputs);
    aiio.updateInputs(aiInputs);
    Logger.processInputs("Vision/Limelight", limelightInputs);
    Logger.processInputs("Vision/AI", aiInputs);

    // We want there to be a new and valid entry before we push updates
    if ((limelightInputs.captureTimestamp != lastTimestampApril) && limelightInputs.validEntry) {
      lastTimestampApril = limelightInputs.captureTimestamp;
      for (BiConsumer<Pose3d, Double> botPoseConsumer : botPoseConsumers) {
        botPoseConsumer.accept(
            limelightInputs.selfPoseFieldSpace, limelightInputs.captureTimestamp);
      }

      for (BiConsumer<Pose3d, Double> targetConsumer : targetConsumers) {
        targetConsumer.accept(
            limelightInputs.targetPoseRobotSpace, limelightInputs.captureTimestamp);
      }
    }

    // We want there to be a new and valid entry before we push updates
    if ((aiInputs.captureTimestamp != lastTimestampAi) && aiInputs.validEntry) {
      lastTimestampAi = aiInputs.captureTimestamp;
      for (BiConsumer<TargetNote, Double> targetNoteconsumer : targetNoteconsumers) {
        targetNoteconsumer.accept(
            targetNoteFactory(
                aiInputs.xError, aiInputs.yError, aiInputs.targetArea, aiInputs.targetClass),
            aiInputs.captureTimestamp);
      }
    } else {
      for (BiConsumer<TargetNote, Double> targetNoteconsumer : targetNoteconsumers) {
        targetNoteconsumer.accept(targetNoteFactory(0, 0, 0, "Null"), 0.0);
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

  public void removeTargetPoseConsumer(BiConsumer<Pose3d, Double> poseConsumer) {
    targetConsumers.remove(poseConsumer);
  }

  public void setVisionPipelineApril(IntSupplier pipeline) {
    limelightIO.setPipeline(pipeline.getAsInt());
  }

  public int getCurrentVisionPipelineApril() {
    return limelightInputs.currentPipeline;
  }

  public void registerTargetNoteConsumer(BiConsumer<TargetNote, Double> targetNoteConsumer) {
    targetNoteconsumers.add(targetNoteConsumer);
  }

  public void removeTargetNoteConsumer(BiConsumer<TargetNote, Double> targetNoteConsumer) {
    targetNoteconsumers.remove(targetNoteConsumer);
  }

  public void setVisionPipelineAI(int pipeline) {
    aiio.setPipeline(pipeline);
  }

  public int getCurrentVisionPipelineAI() {
    return aiInputs.currentPipeline;
  }

  private TargetNote targetNoteFactory(
      double xError, double yError, double targetArea, String targetClass) {
    return new TargetNote(
        new Rotation2d(Units.degreesToRadians(xError)),
        new Rotation2d(Units.degreesToRadians(yError)),
        targetArea,
        targetClass);
  }
}
