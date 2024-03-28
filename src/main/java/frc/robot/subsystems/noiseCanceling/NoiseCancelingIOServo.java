package frc.robot.subsystems.noiseCanceling;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class NoiseCancelingIOServo implements NoiseCancelingIO {
  private final Servo servoOne = new Servo(Constants.NoiseCanceling.SERVO_ONE_PORT);
  private final Servo servoTwo = new Servo(Constants.NoiseCanceling.SERVO_TWO_PORT);

  public NoiseCancelingIOServo() {
    servoOne.setDisabled();
    servoTwo.setDisabled();
    servoOne.setBoundsMicroseconds(0, 0, 0, 0, 0);
    servoTwo.setBoundsMicroseconds(0, 0, 0, 0, 0);
  }

  @Override
  public void setMotorOne(Rotation2d rotation) {
    servoOne.setAngle(rotation.getDegrees());
  }

  @Override
  public void setMotorTwo(Rotation2d rotation) {
    servoTwo.setAngle(rotation.getDegrees());
  }

  @Override
  public void updateInputs(NoiseCancelingInputs inputs) {
    inputs.servoOneRotation = new Rotation2d(servoOne.getAngle());
    inputs.servoTwoRotation = new Rotation2d(servoTwo.getAngle());
  }
}
