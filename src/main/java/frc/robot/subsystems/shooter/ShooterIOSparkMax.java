package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ShooterIOSparkMax implements ShooterIO {

  private final CANSparkMax motorOne =
      new CANSparkMax(
          Constants.Shooter.PUSHER_MOTOR_CAN_ID, Constants.Shooter.SPARK_MAX_MOTOR_ONE_TYPE);
  private final DigitalInput holderLimitSwitch =
      new DigitalInput(Constants.Shooter.HOLDER_LIMIT_SWITCH_PORT);
  private final DigitalInput shotLimitSwitch =
      new DigitalInput(Constants.Shooter.SHOT_LIMIT_SWITCH_PORT);

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
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.holdingLimitSwitch = holderLimitSwitch.get();
    inputs.shotLimitSwitch = shotLimitSwitch.get();
  }

  @Override
  public void setMotorVoltage(double voltage) {
    motorOne.setVoltage(voltage);
  }
}
