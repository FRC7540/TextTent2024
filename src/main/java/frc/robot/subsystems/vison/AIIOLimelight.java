package frc.robot.subsystems.vison;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import java.util.EnumSet;
import org.littletonrobotics.junction.Logger;

public class AIIOLimelight implements AIIO {
  private double captureTimestamp = 0.0;
  private boolean validEntry = false;
  private int currentPipeline = -1;
  private double xError = 0.0;
  private double yError = 0.0;
  private double targetArea = 0.0;
  private String targetClass = "";
  private double stickangle;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable limelight = inst.getTable("limelight-ai");

  private final IntegerEntry pipelineEntry;
  private final DoubleSubscriber captureTimestampSubscriber;
  private final DoubleSubscriber timstampLatentcySubscriber;
  private final DoubleSubscriber xErrorSubscriber;
  private final DoubleSubscriber yErrorSubscriber;
  private final DoubleSubscriber targetAreaSubscriber;
  private final DoubleSubscriber validEntrySubscriber;
  private final IntegerPublisher pipelinePublisher;
  private final StringSubscriber targetClassSubscriber;

  public AIIOLimelight() {
    captureTimestampSubscriber = limelight.getDoubleTopic("cl").subscribe(0.0);
    timstampLatentcySubscriber = limelight.getDoubleTopic("tl").subscribe(0.0);

    xErrorSubscriber = limelight.getDoubleTopic("tx").subscribe(0.0);
    yErrorSubscriber = limelight.getDoubleTopic("ty").subscribe(0.0);
    targetAreaSubscriber = limelight.getDoubleTopic("ta").subscribe(0.0);
    targetClassSubscriber = limelight.getStringTopic("tclass").subscribe("null");

    validEntrySubscriber = limelight.getDoubleTopic("tv").subscribe(0.0);
    pipelineEntry = limelight.getIntegerTopic("getpipe").getEntry(-1);
    pipelinePublisher = limelight.getIntegerTopic("pipeline").publish();

    limelight.addListener(
        EnumSet.of(NetworkTableEvent.Kind.kTopic, NetworkTableEvent.Kind.kValueAll),
        (table, key, event) -> {
          double timestamp =
              Logger.getRealTimestamp()
                  - (captureTimestampSubscriber.get() + timstampLatentcySubscriber.get());
          double xerror = xErrorSubscriber.get();
          double yError = yErrorSubscriber.get();
          double targetArea = targetAreaSubscriber.get();
          String targetClass = targetClassSubscriber.get();
          synchronized (AIIOLimelight.this) {
            captureTimestamp = timestamp;
            this.targetArea = targetArea;
            this.xError = xerror;
            this.yError = yError;
            this.targetClass = targetClass;
            validEntry = validEntrySubscriber.get() == 1.0;
            currentPipeline = (int) pipelineEntry.get();
          }
        });
  }

  @Override
  public synchronized void updateInputs(AIInputs inputs) {
    inputs.captureTimestamp = captureTimestamp;
    inputs.currentPipeline = currentPipeline;
    inputs.validEntry = validEntry;
    inputs.targetArea = targetArea;
    if (xError != stickangle) {
      stickangle = xError;
    }
    inputs.xError = stickangle;
    inputs.yError = yError;
    inputs.targetClass = targetClass;
  }

  @Override
  public void setPipeline(int pipeline) {
    pipelinePublisher.set(pipeline);
  }
}
