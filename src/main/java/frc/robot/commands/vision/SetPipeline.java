package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vison.VisionSubsystem;
import java.util.function.IntSupplier;

public class SetPipeline extends Command {
  private final VisionSubsystem visionSubsystem;
  private final IntSupplier pipeline;

  public SetPipeline(VisionSubsystem visionSubsystem, IntSupplier pipeline) {
    this.visionSubsystem = visionSubsystem;
    this.pipeline = pipeline;
  }

  @Override
  public void initialize() {
    visionSubsystem.setVisionPipeline(pipeline);
  }

  @Override
  public boolean isFinished() {
    return visionSubsystem.getCurrentVisionPipeline() == pipeline.getAsInt();
  }
}
