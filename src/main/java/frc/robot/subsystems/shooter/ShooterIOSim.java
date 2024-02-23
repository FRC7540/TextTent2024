package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

// TODO: Implement shooter sensors
public class ShooterIOSim implements ShooterIO {
  private double firingMotorAppliedVoltage = 0.0;
  private DCMotorSim firingMotorSim =
      new DCMotorSim(
          Constants.Shooter.PUSHER_MOTOR_TYPE,
          Constants.Shooter.PUSHER_MOTOR_GEART_RATIO,
          Constants.Shooter.PUSHER_MOMENT_OF_INTERTIA);

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    firingMotorSim.update(Constants.Shooter.NOMINAL_LOOP_PERIOD);
    inputs.firingMotorAppliedVoltage = firingMotorAppliedVoltage;
    inputs.holdingLimitSwitch = true;
  }

  @Override
  public void setMotorVoltage(double voltage) {
    firingMotorAppliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    firingMotorSim.setInputVoltage(firingMotorAppliedVoltage);
  }
}
