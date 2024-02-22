package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;

public class FlywheelIOSparkMax implements FlywheelIO {
  private final CANSparkMax wheelOneSparkMax =
      new CANSparkMax(Shooter.Flywheel.MOTOR_ONE_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax wheelTwoSparkMax =
      new CANSparkMax(Shooter.Flywheel.MOTOR_TWO_CAN_ID, MotorType.kBrushless);

  private final RelativeEncoder wheelOneEncoder;
  private final RelativeEncoder wheelTwoEncoder;

  public FlywheelIOSparkMax() {

    wheelOneSparkMax.restoreFactoryDefaults();
    wheelTwoSparkMax.restoreFactoryDefaults();

    wheelOneSparkMax.setCANTimeout(250);
    wheelTwoSparkMax.setCANTimeout(250);

    wheelOneEncoder = wheelOneSparkMax.getEncoder();
    wheelTwoEncoder = wheelTwoSparkMax.getEncoder();

    wheelOneSparkMax.setInverted(Shooter.Flywheel.WheelOne.INVERTED);
    wheelTwoSparkMax.setInverted(Shooter.Flywheel.WheelTwo.INVERTED);
    wheelOneSparkMax.setSmartCurrentLimit(20);
    wheelTwoSparkMax.setSmartCurrentLimit(20);
    wheelOneSparkMax.enableVoltageCompensation(12.0);
    wheelTwoSparkMax.enableVoltageCompensation(12.0);

    wheelOneEncoder.setPosition(0.0);
    wheelOneEncoder.setMeasurementPeriod(10);
    wheelOneEncoder.setAverageDepth(8);

    wheelTwoEncoder.setPosition(0.0);
    wheelTwoEncoder.setMeasurementPeriod(10);
    wheelTwoEncoder.setAverageDepth(8);

    wheelOneEncoder.setPositionConversionFactor(
        (2 * Math.PI) / Constants.Shooter.Flywheel.WheelOne.GEAR_RATIO);
    wheelTwoEncoder.setPositionConversionFactor(
        (2 * Math.PI) / Constants.Shooter.Flywheel.WheelTwo.GEAR_RATIO);

    wheelOneEncoder.setVelocityConversionFactor(
        ((2 * Math.PI) / Constants.Shooter.Flywheel.WheelOne.GEAR_RATIO) / 60);
    wheelTwoEncoder.setVelocityConversionFactor(
        ((2 * Math.PI) / Constants.Shooter.Flywheel.WheelTwo.GEAR_RATIO) / 60);

    wheelOneSparkMax.setCANTimeout(0);
    wheelTwoSparkMax.setCANTimeout(0);

    wheelOneSparkMax.burnFlash();
    wheelTwoSparkMax.burnFlash();
  }

  @Override
  public void setWheelOneVoltage(double volts) {
    wheelOneSparkMax.setVoltage(volts);
  }

  @Override
  public void setWheelTwoVoltage(double volts) {
    wheelTwoSparkMax.setVoltage(volts);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {

    inputs.wheelOneRadSec = wheelOneEncoder.getVelocity();
    inputs.wheelOnePositionRad = wheelOneEncoder.getPosition();
    inputs.wheelOneAppliedAmps = wheelOneSparkMax.getOutputCurrent();
    inputs.wheelOneAppliedVolts =
        wheelOneSparkMax.getAppliedOutput() * wheelOneSparkMax.getBusVoltage();

    inputs.wheelTwoRadSec = wheelTwoEncoder.getVelocity();
    inputs.wheelTwoPositionRad = wheelTwoEncoder.getPosition();
    inputs.wheelTwoAppliedAmps = wheelTwoSparkMax.getOutputCurrent();
    inputs.wheelTwoAppliedVolts =
        wheelTwoSparkMax.getAppliedOutput() * wheelTwoSparkMax.getBusVoltage();
  }
}
