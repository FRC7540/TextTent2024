package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends SubsystemBase {

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private double wheelOneTargetSpeed = 0.0;
  private double wheelTwoTargetSpeed = 0.0;

  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    io.setWheelOneVoltage(wheelOneTargetSpeed);
    io.setWheelTwoVoltage(wheelTwoTargetSpeed);
  }

  @Override
  public void simulationPeriodic() {}

  public void setBothFlywheelSpeeds(double speed) {
    wheelOneTargetSpeed = speed;
    wheelTwoTargetSpeed = speed;
  }

  public void stop() {
    io.setWheelOneVoltage(0.0);
    io.setWheelTwoVoltage(0.0);
  }
}
