package frc.robot.subsystems.flywheel;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class FlywheelIOSparkMax implements FlywheelIO {
  private final CANSparkMax wheelOneSparkMax =
      new CANSparkMax(Constants.Flywheel.MOTOR_ONE_CAN_ID, MotorType.kBrushless);
  private final CANSparkMax wheelTwoSparkMax =
      new CANSparkMax(Constants.Flywheel.MOTOR_TWO_CAN_ID, MotorType.kBrushless);

  private final RelativeEncoder wheelOneEncoder;
  private final RelativeEncoder wheelTwoEncoder;

  private boolean iswheelOneInverted = false;
  private boolean iswheelTwoInverted = true;

  public FlywheelIOSparkMax() {

    // wheelOneSparkMax.restoreFactoryDefaults();
    // wheelTwoSparkMax.restoreFactoryDefaults();

    // wheelOneSparkMax.setCANTimeout(250);
    // wheelTwoSparkMax.setCANTimeout(250);

    wheelOneEncoder = wheelOneSparkMax.getEncoder();
    wheelTwoEncoder = wheelTwoSparkMax.getEncoder();

    wheelOneSparkMax.setInverted(iswheelOneInverted);
    wheelTwoSparkMax.setInverted(iswheelTwoInverted);
    // wheelOneSparkMax.setSmartCurrentLimit(40);
    // wheelTwoSparkMax.setSmartCurrentLimit(30);
    // wheelOneSparkMax.enableVoltageCompensation(12.0);
    // wheelTwoSparkMax.enableVoltageCompensation(12.0);

    wheelOneEncoder.setPosition(0.0);
    wheelOneEncoder.setMeasurementPeriod(10);
    wheelOneEncoder.setAverageDepth(2);

    wheelTwoEncoder.setPosition(0.0);
    wheelTwoEncoder.setMeasurementPeriod(10);
    wheelTwoEncoder.setAverageDepth(2);

    wheelOneEncoder.setPositionConversionFactor(1 / 4);
    wheelTwoEncoder.setPositionConversionFactor(1 / 4);

    // wheelOneSparkMax.setCANTimeout(0);
    // wheelTwoSparkMax.setCANTimeout(0);

    // wheelOneSparkMax.burnFlash();
    // wheelTwoSparkMax.burnFlash();
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

    inputs.wheelOneRadSec =
        Units.rotationsPerMinuteToRadiansPerSecond(wheelOneEncoder.getVelocity());
    inputs.wheelTwoRadSec =
        Units.rotationsPerMinuteToRadiansPerSecond(wheelTwoEncoder.getVelocity());
  }
}
