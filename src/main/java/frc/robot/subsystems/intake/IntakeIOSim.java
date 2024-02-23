package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

// TODO: Implement intake sensors
public class IntakeIOSim implements IntakeIO {
  private double motorOneVolts = 0.0;
  private DCMotorSim motorOneSim =
      new DCMotorSim(
          Constants.Intake.MOTOR_TYPE,
          Constants.Intake.MOTOR_GEAR_RATIO,
          Constants.Intake.MOTOR_MOMENT_OF_INTERTIA);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    motorOneSim.update(Constants.Shooter.NOMINAL_LOOP_PERIOD);
    inputs.motorAppliedVoltage = motorOneVolts;
  }

  @Override
  public void setVoltage(double voltage) {
    motorOneVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    motorOneSim.setInputVoltage(motorOneVolts);
  }
}
