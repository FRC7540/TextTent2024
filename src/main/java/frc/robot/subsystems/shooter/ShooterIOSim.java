package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
  private double pusherMotorAppliedVoltage = 0.0;
  private DCMotorSim pusherMotorSim =
      new DCMotorSim(
          Constants.Shooter.PUSHER_MOTOR_TYPE,
          Constants.Shooter.PUSHER_MOTOR_GEART_RATIO,
          Constants.Shooter.PUSHER_MOMENT_OF_INTERTIA);

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    pusherMotorSim.update(Constants.Shooter.NOMINAL_LOOP_PERIOD);
    inputs.pusherMotorAppliedVoltage = pusherMotorAppliedVoltage;
  }

  @Override
  public void setMotorVoltage(double voltage) {
    pusherMotorAppliedVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
    pusherMotorSim.setInputVoltage(pusherMotorAppliedVoltage);
  }
}
