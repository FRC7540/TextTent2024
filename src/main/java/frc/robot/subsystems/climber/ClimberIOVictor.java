package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ClimberIOVictor implements ClimberIO {

  VictorSPX climberController = new VictorSPX(Constants.Climber.CLIMBER_MOTOR_CONTROLLER_CAN_ID);
  DutyCycleEncoder climberEncoder = new DutyCycleEncoder(Constants.Climber.CLIMBER_ENCODER_PORT);

  public ClimberIOVictor() {
    climberEncoder.setDistancePerRotation(Constants.Climber.CLIMBER_ENCODER_DISTANCE_PER_ROTATION);
    climberEncoder.reset();
    climberController.set(VictorSPXControlMode.Disabled, 0.0);
  }

  @Override
  public void driveMotor(double voltage) {
    climberController.set(
        VictorSPXControlMode.PercentOutput, MathUtil.clamp(voltage, -12, 12) / 12);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberSpoolRad = climberEncoder.getDistance();
    inputs.motorAppliedVoltage = climberController.getMotorOutputVoltage();
  }
}
