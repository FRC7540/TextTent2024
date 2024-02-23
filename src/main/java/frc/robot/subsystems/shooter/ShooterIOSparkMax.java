package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

// TODO: Implement shooter sensors
public class ShooterIOSparkMax implements ShooterIO {

  private final CANSparkMax firingMotor =
      new CANSparkMax(
          Constants.Shooter.PUSHER_MOTOR_CAN_ID, Constants.Shooter.SPARK_MAX_MOTOR_ONE_TYPE);
  private final DigitalInput holderLimitSwitch =
      new DigitalInput(Constants.Shooter.HOLDER_LIMIT_SWITCH_PORT);
  private final DigitalInput shotLimitSwitch =
      new DigitalInput(Constants.Shooter.SHOT_LIMIT_SWITCH_PORT);

  private double firingMotorApplliedVoltage = 0.0;

  public ShooterIOSparkMax() {
    firingMotor.restoreFactoryDefaults();
    firingMotor.setCANTimeout(250);
    firingMotor.enableVoltageCompensation(12.0);
    firingMotor.setSmartCurrentLimit(25);
    firingMotor.setInverted(Constants.Shooter.PUSHER_MOTOR_INVERTED);
    firingMotor.setIdleMode(IdleMode.kCoast);
    firingMotor.setCANTimeout(0);
    firingMotor.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.holdingLimitSwitch = holderLimitSwitch.get();
    inputs.shotLimitSwitch = shotLimitSwitch.get();
    inputs.firingMotorAppliedVoltage = firingMotorApplliedVoltage;
    inputs.holdingLimitSwitch = true;
  }

  @Override
  public void setMotorVoltage(double voltage) {
    firingMotorApplliedVoltage = voltage;
    firingMotor.setVoltage(voltage);
  }
}
