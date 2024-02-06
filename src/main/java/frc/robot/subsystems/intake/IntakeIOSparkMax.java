package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

public class IntakeIOSparkMax implements IntakeIO {

  private final CANSparkMax motor =
      new CANSparkMax(Constants.Intake.MOTOR_ONE_CAN_ID, Constants.Intake.SPARK_MOTOR_TYPE);

  public IntakeIOSparkMax() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {}

  @Override
  public void driveMotor(double voltage) {
    motor.setVoltage(voltage);
  }
}
