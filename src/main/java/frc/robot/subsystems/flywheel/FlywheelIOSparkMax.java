package frc.robot.subsystems.flywheel;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

public class FlywheelIOSparkMax implements FlywheelIO {
  private final CANSparkMax wheelOneSparkMax =
      new CANSparkMax(Constants.Flywheel.MOTOR_ONE_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax wheelTwoSparkMax =
      new CANSparkMax(Constants.Flywheel.MOTOR_TWO_CAN_ID, MotorType.kBrushless);

  private final RelativeEncoder wheelOneEncoder;
  private final RelativeEncoder wheelTwoEncoder;

  private double wheelOneAppliedVolts = 0.0;
  private double wheelTwoAppliedVolts = 0.0;

  public FlywheelIOSparkMax() {

    wheelOneSparkMax.restoreFactoryDefaults();
    wheelTwoSparkMax.restoreFactoryDefaults();

    wheelOneSparkMax.setCANTimeout(250);
    wheelTwoSparkMax.setCANTimeout(250);

    wheelOneEncoder = wheelOneSparkMax.getEncoder();
    wheelTwoEncoder = wheelTwoSparkMax.getEncoder();

    wheelOneSparkMax.setInverted(Constants.Flywheel.WheelOne.INVERTED);
    wheelTwoSparkMax.setInverted(Constants.Flywheel.WheelTwo.INVERTED);
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

    wheelOneEncoder.setPositionConversionFactor(1 / 2 * Math.PI);
    wheelTwoEncoder.setPositionConversionFactor(1 / 2 * Math.PI);

    wheelOneEncoder.setVelocityConversionFactor(0.1047);
    wheelTwoEncoder.setVelocityConversionFactor(0.1047);

    wheelOneSparkMax.setCANTimeout(0);
    wheelTwoSparkMax.setCANTimeout(0);

    wheelOneSparkMax.burnFlash();
    wheelTwoSparkMax.burnFlash();
  }

  @Override
  public void setWheelOneVoltage(double volts) {
    wheelOneAppliedVolts = volts;
    wheelOneSparkMax.setVoltage(volts);
  }

  @Override
  public void setWheelTwoVoltage(double volts) {
    wheelTwoAppliedVolts = volts;
    wheelTwoSparkMax.setVoltage(volts);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {

    inputs.wheelOneRadSec = wheelOneEncoder.getVelocity() * 0.25;
    inputs.wheelOnePositionRad = wheelOneEncoder.getPosition() * 0.25;
    inputs.wheelOneAppliedAmps = wheelOneSparkMax.getOutputCurrent();
    inputs.wheelOneAppliedVolts = wheelOneSparkMax.getAppliedOutput();

    inputs.wheelTwoRadSec = wheelTwoEncoder.getVelocity() * 0.25;
    inputs.wheelTwoPositionRad = wheelTwoEncoder.getPosition() * 0.25;
    inputs.wheelTwoAppliedAmps = wheelTwoSparkMax.getOutputCurrent();
    inputs.wheelTwoAppliedVolts = wheelTwoSparkMax.getAppliedOutput();
  }
}
