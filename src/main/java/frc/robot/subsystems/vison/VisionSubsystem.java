package frc.robot.subsystems.vison;

import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends VirtualSubsystem {

  private final LimelightIO limelightIO;
  private final LimelightIOInputsAutoLogged limelightInputs = new LimelightIOInputsAutoLogged();

  public VisionSubsystem(LimelightIO limelightIO) {
    this.limelightIO = limelightIO;
  }

  @Override
  public void periodic() {
    limelightIO.updateInputs(limelightInputs);
    Logger.processInputs("Vision/Limelight", limelightInputs);
  }
}
