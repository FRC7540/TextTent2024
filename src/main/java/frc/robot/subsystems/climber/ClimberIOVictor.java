package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ClimberIOVictor implements ClimberIO {

  VictorSPX climberController = new VictorSPX(16);
  DutyCycleEncoder climberEncoder = new DutyCycleEncoder(4);

  public ClimberIOVictor() {
    climberEncoder.setDistancePerRotation(-1);
    climberEncoder.reset();
    climberController.set(VictorSPXControlMode.Disabled, 0.0);
  }

  @Override
  public void driveMotor(double voltage) {
    climberController.set(
        VictorSPXControlMode.PercentOutput, -1 * MathUtil.clamp(voltage, -12, 12) / 12);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberSpoolRad = climberEncoder.getDistance();
    inputs.motorAppliedVoltage = climberController.getMotorOutputVoltage();
  }
}
