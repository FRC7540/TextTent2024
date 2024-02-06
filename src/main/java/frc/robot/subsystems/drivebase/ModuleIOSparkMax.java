package frc.robot.subsystems.drivebase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AbsoluteEncoder turnAbsoluteEncoder;

  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0:
        driveSparkMax =
            new CANSparkMax(Constants.Drivebase.Mod0.DRIVE_SPARKMAX_CAN_ID, MotorType.kBrushless);
        turnSparkMax =
            new CANSparkMax(Constants.Drivebase.Mod0.TURN_SPARKMAX_CAN_ID, MotorType.kBrushless);
        absoluteEncoderOffset = Constants.Drivebase.Mod0.ABSOULUTE_OFFSET;
        turnSparkMax.setInverted(Constants.Drivebase.Mod0.TURN_MOTOR_INVERT);
        break;
      case 1:
        driveSparkMax =
            new CANSparkMax(Constants.Drivebase.Mod1.DRIVE_SPARKMAX_CAN_ID, MotorType.kBrushless);
        turnSparkMax =
            new CANSparkMax(Constants.Drivebase.Mod1.TURN_SPARKMAX_CAN_ID, MotorType.kBrushless);
        absoluteEncoderOffset = Constants.Drivebase.Mod1.ABSOULUTE_OFFSET;
        turnSparkMax.setInverted(Constants.Drivebase.Mod1.TURN_MOTOR_INVERT);
        break;
      case 2:
        driveSparkMax =
            new CANSparkMax(Constants.Drivebase.Mod2.DRIVE_SPARKMAX_CAN_ID, MotorType.kBrushless);
        turnSparkMax =
            new CANSparkMax(Constants.Drivebase.Mod2.TURN_SPARKMAX_CAN_ID, MotorType.kBrushless);
        absoluteEncoderOffset = Constants.Drivebase.Mod2.ABSOULUTE_OFFSET;
        turnSparkMax.setInverted(Constants.Drivebase.Mod2.TURN_MOTOR_INVERT);
        break;
      case 3:
        driveSparkMax =
            new CANSparkMax(Constants.Drivebase.Mod3.DRIVE_SPARKMAX_CAN_ID, MotorType.kBrushless);
        turnSparkMax =
            new CANSparkMax(Constants.Drivebase.Mod3.TURN_SPARKMAX_CAN_ID, MotorType.kBrushless);
        absoluteEncoderOffset = Constants.Drivebase.Mod3.ABSOULUTE_OFFSET;
        turnSparkMax.setInverted(Constants.Drivebase.Mod3.TURN_MOTOR_INVERT);
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    turnAbsoluteEncoder.setInverted(true);
    turnRelativeEncoder.setInverted(true);

    turnAbsoluteEncoder.setPositionConversionFactor(2 * Math.PI);
    turnAbsoluteEncoder.setVelocityConversionFactor((2 * Math.PI) / 60.0);

    turnRelativeEncoder.setPositionConversionFactor(2 * Math.PI);
    turnRelativeEncoder.setVelocityConversionFactor((2 * Math.PI) / 60.0);

    driveEncoder.setPositionConversionFactor((0.0762 * Math.PI));
    driveEncoder.setVelocityConversionFactor(
        ((0.0762 * Math.PI) / ((45.0 * 22) / (14 * 15))) / 60.0);

    driveSparkMax.setSmartCurrentLimit(40);
    turnSparkMax.setSmartCurrentLimit(25);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(8);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(8);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = driveEncoder.getPosition();
    inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition = new Rotation2d(turnAbsoluteEncoder.getPosition()).plus(absoluteEncoderOffset);
    inputs.turnPosition = new Rotation2d(turnRelativeEncoder.getPosition());

    inputs.turnVelocityRadPerSec = turnRelativeEncoder.getVelocity();
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
