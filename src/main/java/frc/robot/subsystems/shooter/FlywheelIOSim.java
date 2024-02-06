package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.Shooter;

public class FlywheelIOSim implements FlywheelIO {
  private FlywheelSim wheelOneSim =
      new FlywheelSim(
          DCMotor.getNEO(Shooter.Flywheel.WheelOne.MOTOR_COUNT),
          Shooter.Flywheel.WheelOne.GEAR_RATIO,
          Shooter.Flywheel.WheelOne.MOMENT_OF_INERTIA);
  private FlywheelSim wheelTwoSim =
      new FlywheelSim(
          DCMotor.getNEO(Shooter.Flywheel.WheelTwo.MOTOR_COUNT),
          Shooter.Flywheel.WheelTwo.GEAR_RATIO,
          Shooter.Flywheel.WheelTwo.MOMENT_OF_INERTIA);

  private double motorOneAppliedVolts = 0.0;
  private double motorTwoAppliedVolts = 0.0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    wheelOneSim.update(Shooter.Flywheel.SIM_UPDATE_TIME);
    wheelTwoSim.update(Shooter.Flywheel.SIM_UPDATE_TIME);

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
    motorOneAppliedVolts = volts;
  }

  @Override
  public void setWheelTwoVoltage(double volts) {
    wheelTwoSim.setInputVoltage(volts);
    motorTwoAppliedVolts = volts;
  }
}
