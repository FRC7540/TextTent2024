package frc.robot.subsystems.noiseCanceling;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.States.NoiseCancelingState;
import org.littletonrobotics.junction.Logger;

public class NoiseCancelingSubsystem extends SubsystemBase {
  public NoiseCancelingIO noiseCancelingIO;
  public NoiseCancelingInputsAutoLogged noiseCancelingInputs = new NoiseCancelingInputsAutoLogged();

  private NoiseCancelingState currentState;

  public NoiseCancelingSubsystem(NoiseCancelingIO noiseCancelingIO) {
    this.noiseCancelingIO = noiseCancelingIO;
  }

  @Override
  public void periodic() {
    noiseCancelingIO.updateInputs(noiseCancelingInputs);
    Logger.processInputs("NoiseCanceling", noiseCancelingInputs);
    determineState();
  }

  public NoiseCancelingState getState() {
    return currentState;
  }

  private void determineState() {
    if (!servoOneDeployed() && !servoTwoDeployed()) {
      currentState = NoiseCancelingState.READY;
      return;
    }

    if (servoOneDeployed() && servoTwoDeployed()) {
      currentState = NoiseCancelingState.DEPLOYED_BOTH;
      return;
    }

    if (servoOneDeployed() && !servoTwoDeployed()) {
      currentState = NoiseCancelingState.DEPLOYED_ONE;
      return;
    }

    if (servoTwoDeployed() && !servoOneDeployed()) {
      currentState = NoiseCancelingState.DEPLOYED_TWO;
      return;
    }
  }

  private boolean servoOneDeployed() {
    return noiseCancelingInputs.servoOneRotation.getRadians()
        >= Constants.NoiseCanceling.SERVO_ONE_DEPLOYED_THRESHOLD.getRadians();
  }

  private boolean servoTwoDeployed() {
    return noiseCancelingInputs.servoTwoRotation.getRadians()
        >= Constants.NoiseCanceling.SERVO_TWO_DEPLOYED_THRESHOLD.getRadians();
  }
}
