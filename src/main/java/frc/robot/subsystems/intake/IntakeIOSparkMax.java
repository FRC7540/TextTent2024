package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

// TODO: Implement intake sensors
public class IntakeIOSparkMax implements IntakeIO {

  private final CANSparkMax motorOne =
      new CANSparkMax(Constants.Intake.MOTOR_ONE_CAN_ID, MotorType.kBrushed);

  private final DigitalInput intakeNoteLimtitSwitch =
      new DigitalInput(Constants.Intake.INTAKE_NOTE_LIMIT_SWITCH_PORT);

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
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeNoteSwitch = intakeNoteLimtitSwitch.get();
  }

  @Override
  public void setVoltage(double voltage) {
    motorOne.setVoltage(voltage);
  }
}
