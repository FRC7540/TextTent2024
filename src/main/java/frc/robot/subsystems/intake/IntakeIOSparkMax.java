package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

public class IntakeIOSparkMax implements IntakeIO {

  private final CANSparkMax motorOne =
      new CANSparkMax(Constants.Intake.MOTOR_ONE_CAN_ID, Constants.Intake.SPARK_MOTOR_ONE_TYPE);

  public IntakeIOSparkMax() {
    motorOne.restoreFactoryDefaults();
    motorOne.setCANTimeout(250);
    motorOne.enableVoltageCompensation(12.0);
    motorOne.setSmartCurrentLimit(25);
    motorOne.setInverted(Constants.Intake.MOTOR_ONE_INVERTED);
    motorOne.setIdleMode(IdleMode.kBrake);
    motorOne.setCANTimeout(0);
    motorOne.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {}

  @Override
  public void driveMotor(double voltage) {
    motorOne.setVoltage(voltage);
  }
}
