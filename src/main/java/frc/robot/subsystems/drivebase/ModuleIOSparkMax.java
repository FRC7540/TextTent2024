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
            new CANSparkMax(Constants.Drivebase.ModFL.DRIVE_SPARKMAX_CAN_ID, MotorType.kBrushless);
        turnSparkMax =
            new CANSparkMax(Constants.Drivebase.ModFL.TURN_SPARKMAX_CAN_ID, MotorType.kBrushless);
        absoluteEncoderOffset = Constants.Drivebase.ModFL.ABSOULUTE_OFFSET;
        turnSparkMax.setInverted(Constants.Drivebase.ModFL.TURN_MOTOR_INVERT);
        break;
      case 1:
        driveSparkMax =
            new CANSparkMax(Constants.Drivebase.ModFR.DRIVE_SPARKMAX_CAN_ID, MotorType.kBrushless);
        turnSparkMax =
            new CANSparkMax(Constants.Drivebase.ModFR.TURN_SPARKMAX_CAN_ID, MotorType.kBrushless);
        absoluteEncoderOffset = Constants.Drivebase.ModFR.ABSOULUTE_OFFSET;
        turnSparkMax.setInverted(Constants.Drivebase.ModFR.TURN_MOTOR_INVERT);
        break;
      case 2:
        driveSparkMax =
            new CANSparkMax(Constants.Drivebase.ModBL.DRIVE_SPARKMAX_CAN_ID, MotorType.kBrushless);
        turnSparkMax =
            new CANSparkMax(Constants.Drivebase.ModBL.TURN_SPARKMAX_CAN_ID, MotorType.kBrushless);
        absoluteEncoderOffset = Constants.Drivebase.ModBL.ABSOULUTE_OFFSET;
        turnSparkMax.setInverted(Constants.Drivebase.ModBL.TURN_MOTOR_INVERT);

        break;
      case 3:
        driveSparkMax =
            new CANSparkMax(Constants.Drivebase.ModBR.DRIVE_SPARKMAX_CAN_ID, MotorType.kBrushless);
        turnSparkMax =
            new CANSparkMax(Constants.Drivebase.ModBR.TURN_SPARKMAX_CAN_ID, MotorType.kBrushless);
        absoluteEncoderOffset = Constants.Drivebase.ModBR.ABSOULUTE_OFFSET;
        turnSparkMax.setInverted(Constants.Drivebase.ModBR.TURN_MOTOR_INVERT);
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
    // turnRelativeEncoder.setInverted(true);

    // turnSparkMax.setInverted(true);
    turnAbsoluteEncoder.setPositionConversionFactor(2 * Math.PI);
    turnAbsoluteEncoder.setVelocityConversionFactor((2 * Math.PI) / 60.0);

    turnRelativeEncoder.setPositionConversionFactor(2 * Math.PI);
    turnRelativeEncoder.setVelocityConversionFactor((2 * Math.PI) / 60.0);

    driveEncoder.setPositionConversionFactor(
        (Constants.Drivebase.WHEEL_DIAMETER * Math.PI) / Constants.Drivebase.DRIVE_GEAR_RATIO);
    driveEncoder.setVelocityConversionFactor(
        ((Constants.Drivebase.WHEEL_DIAMETER * Math.PI) / Constants.Drivebase.DRIVE_GEAR_RATIO)
            / 60.0);

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

    inputs.turnAbsolutePosition =
        new Rotation2d(turnAbsoluteEncoder.getPosition()).plus(absoluteEncoderOffset);
    inputs.turnPosition =
        new Rotation2d(turnAbsoluteEncoder.getPosition()).plus(absoluteEncoderOffset);

    inputs.turnVelocityRadPerSec = turnAbsoluteEncoder.getVelocity();
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
