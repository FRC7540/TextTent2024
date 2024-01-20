package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends SubsystemBase {

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private final PIDController wheelOneController;
  private final PIDController wheelTwoController;

  SimpleMotorFeedforward motorOnefeedforward = new SimpleMotorFeedforward(0.1, 0.05);
  SimpleMotorFeedforward motorTwofeedforward = new SimpleMotorFeedforward(0.1, 0.05);

  private Double flywheelSpeedSetpoint = 0.5;

  public FlywheelSubsystem(FlywheelIO io) {

    this.io = io;

    wheelOneController = new PIDController(0.05, 0.0, 0);
    wheelTwoController = new PIDController(0.05, 0.0, 0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (flywheelSpeedSetpoint != null) {
      io.setWheelOneVoltage(motorOnefeedforward.calculate(flywheelSpeedSetpoint));
      io.setWheelTwoVoltage(motorTwofeedforward.calculate(flywheelSpeedSetpoint));
      // io.setWheelOneVoltage(flywheelSpeedSetpoint);
      // io.setWheelTwoVoltage(flywheelSpeedSetpoint);
    }
  }

  @Override
  public void simulationPeriodic() {}

  public void setBothFlywheelSpeeds(double speed) {
    flywheelSpeedSetpoint = speed;
  }

  public void stop() {
    flywheelSpeedSetpoint = 0.0;
    io.setWheelOneVoltage(0.0);
    io.setWheelTwoVoltage(0.0);
  }
}
