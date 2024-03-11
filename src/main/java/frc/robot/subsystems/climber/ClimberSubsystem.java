package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.States.ClimberState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  @AutoLogOutput(key = "Climber/ClimberState")
  private ClimberState climberState;

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    determineClimberState();
    RobotState.climberState = climberState;
  }

  private void determineClimberState() {
    if (getClimberExtension() == 0.0 && getClimberMotorVoltage() == 0.0) {
      climberState = ClimberState.READY;
      return;
    }

    if (((getClimberExtension()
                >= Constants.Climber.EXTENSION_RADIANS - Constants.Climber.EXTENSION_THRESHOLD)
            || (getClimberExtension()
                <= Constants.Climber.EXTENSION_RADIANS + Constants.Climber.EXTENSION_THRESHOLD))
        && getClimberMotorVoltage() == 0.0) {
      climberState = ClimberState.EXTENDED;
      return;
    }

    if (getClimberExtension()
        >= Constants.Climber.EXTENSION_RADIANS * 2 + Constants.Climber.EXTENSION_THRESHOLD) {
      climberState = ClimberState.RETRACTED;
      return;
    }

    if ((getClimberExtension() > 0
            && getClimberExtension()
                < Constants.Climber.EXTENSION_RADIANS - Constants.Climber.EXTENSION_THRESHOLD)
        && getClimberMotorVoltage() > 0) {
      climberState = ClimberState.EXTENDING;
      return;
    }

    if ((getClimberExtension() > 0 && getClimberExtension() > Constants.Climber.EXTENSION_RADIANS)
        && getClimberMotorVoltage() > 0) {
      climberState = ClimberState.RETRACTING;
      return;
    }
    climberState = ClimberState.UNDEFINED;
  }

  private double getClimberMotorVoltage() {
    return inputs.motorAppliedVoltage;
  }

  public Rotation2d getClimberRotations() {
    return new Rotation2d(inputs.climberSpoolRad);
  }

  public double getClimberExtension() {
    return inputs.climberSpoolRad;
  }

  public void setClimberMotorVoltage(double voltage) {
    if (voltage < 0.0) {
      DriverStation.reportError("Cannot drive climber in reverse, hardware incompatible", true);
      return;
    }

    if (climberState == ClimberState.RETRACTED) {
      DriverStation.reportWarning(
          "Climber has alrdy retracted, and this is a one time operation! Reset the robot hardware before trying to run again.",
          false);
      return;
    }
    io.driveMotor(voltage);
  }

  public ClimberState getState() {
    return climberState;
  }
}
