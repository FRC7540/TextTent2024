package frc.robot.subsystems.vison;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.EnumSet;
import org.littletonrobotics.junction.Logger;

public class LimelightIO implements VisionIO {
  private double captureTimestamp = 0.0;
  private Pose3d targetPoseRobotSpace = new Pose3d();
  private Pose3d selfBotPose3d = new Pose3d();
  private boolean validEntry = false;
  private int currentPipeline = -1;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable limelight = inst.getTable("limelight");

  private final IntegerEntry pipelineEntry;
  private final DoubleArraySubscriber selfBotPoseSubscriber;
  private final DoubleArraySubscriber targetPoseRobotSpaceSubscriber;
  private final DoubleSubscriber validEntrySubscriber;
  private final IntegerPublisher pipelinePublisher;

  public LimelightIO() {
    selfBotPoseSubscriber =
        limelight.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[6]);
    targetPoseRobotSpaceSubscriber =
        limelight.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[6]);
    validEntrySubscriber = limelight.getDoubleTopic("tv").subscribe(0.0);
    pipelineEntry = limelight.getIntegerTopic("getpipe").getEntry(-1);
    pipelinePublisher = limelight.getIntegerTopic("pipeline").publish();

    limelight.addListener(
        EnumSet.of(NetworkTableEvent.Kind.kTopic, NetworkTableEvent.Kind.kValueAll),
        (table, key, event) -> {
          double timestamp = Logger.getRealTimestamp() - selfBotPoseSubscriber.get()[6];
          synchronized (LimelightIO.this) {
            captureTimestamp = timestamp;
            selfBotPose3d =
                new Pose3d(
                    new Translation3d(
                        selfBotPoseSubscriber.get()[0],
                        selfBotPoseSubscriber.get()[1],
                        selfBotPoseSubscriber.get()[2]),
                    new Rotation3d(
                        Units.degreesToRadians(selfBotPoseSubscriber.get()[3]),
                        Units.degreesToRadians(selfBotPoseSubscriber.get()[4]),
                        Units.degreesToRadians(selfBotPoseSubscriber.get()[5])));
            targetPoseRobotSpace =
                new Pose3d(
                    new Translation3d(
                        targetPoseRobotSpaceSubscriber.get()[0],
                        targetPoseRobotSpaceSubscriber.get()[1],
                        targetPoseRobotSpaceSubscriber.get()[2]),
                    new Rotation3d(
                        Units.degreesToRadians(targetPoseRobotSpaceSubscriber.get()[3]),
                        Units.degreesToRadians(targetPoseRobotSpaceSubscriber.get()[4]),
                        Units.degreesToRadians(targetPoseRobotSpaceSubscriber.get()[5])));
            validEntry = validEntrySubscriber.get() == 1.0;
            currentPipeline = (int) pipelineEntry.get();
          }
        });
  }

  @Override
  public synchronized void updateInputs(VisionIOInputs inputs) {
    inputs.captureTimestamp = captureTimestamp;
    inputs.currentPipeline = currentPipeline;
    inputs.validEntry = validEntry;
    inputs.selfPoseFieldSpace = selfBotPose3d;
    inputs.targetPoseRobotSpace = targetPoseRobotSpace;
  }

  @Override
  public void setPipeline(int pipeline) {
    pipelinePublisher.set(pipeline);
  }
}
