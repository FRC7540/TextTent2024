package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

public class ShooterIOSparkMax implements ShooterIO {

  private final CANSparkMax motorOne =
      new CANSparkMax(
          Constants.Shooter.PUSHER_MOTOR_CAN_ID, Constants.Shooter.SPARK_MAX_MOTOR_ONE_TYPE);

  public ShooterIOSparkMax() {
    motorOne.restoreFactoryDefaults();
    motorOne.setCANTimeout(250);
    motorOne.enableVoltageCompensation(12.0);
    motorOne.setSmartCurrentLimit(25);
    motorOne.setInverted(Constants.Shooter.PUSHER_MOTOR_INVERTED);
    motorOne.setIdleMode(IdleMode.kCoast);
    motorOne.setCANTimeout(0);
    motorOne.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {}

  @Override
  public void setMotorVoltage(double voltage) {
    motorOne.setVoltage(voltage);
  }
}
