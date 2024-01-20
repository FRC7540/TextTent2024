package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class FlywheelIOSim implements FlywheelIO {
  private FlywheelSim wheelOneSim =
      new FlywheelSim(
          DCMotor.getNEO(1),
          Constants.Flywheel.FLYWHEEL_GEAR_RATIO,
          Constants.Flywheel.FLYWHEEL_MOMENT_OF_INERTIA);
  private FlywheelSim wheelTwoSim =
      new FlywheelSim(
          DCMotor.getNEO(1),
          Constants.Flywheel.FLYWHEEL_GEAR_RATIO,
          Constants.Flywheel.FLYWHEEL_MOMENT_OF_INERTIA);

  private double motorOneAppliedVolts = 0.0;
  private double motorTwoAppliedVolts = 0.0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    wheelOneSim.update(0.02);
    wheelTwoSim.update(0.2);

    inputs.wheelOneRadSec = wheelOneSim.getAngularVelocityRadPerSec();
    inputs.wheelOnePositionRad = 0.0;
    inputs.wheelOneAppliedVolts = motorOneAppliedVolts;
    inputs.wheelOneAppliedAmps = wheelOneSim.getCurrentDrawAmps();

    inputs.wheelTwoRadSec = wheelTwoSim.getAngularVelocityRadPerSec();
    inputs.wheelTwoPositionRad = 0.0;
    inputs.wheelTwoAppliedVolts = motorTwoAppliedVolts;
    inputs.wheelTwoAppliedAmps = wheelTwoSim.getCurrentDrawAmps();
  }

  @Override
  public void setWheelOneVoltage(double volts) {
    wheelOneSim.setInputVoltage(volts);
  }

  @Override
  public void setWheelTwoVoltage(double volts) {
    wheelTwoSim.setInputVoltage(volts);
  }
}
