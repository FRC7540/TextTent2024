package frc.robot.subsystems.drivebase;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;

public class GyroIONavX implements GyroIO {
  AHRS ahrs;

  public GyroIONavX() {
    ahrs = new AHRS(I2C.Port.kMXP);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = ahrs.isConnected();
    inputs.yawPosition = new Rotation2d(Units.degreesToRadians(ahrs.getFusedHeading()));
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(ahrs.getRate());
    inputs.quaternion[0] = ahrs.getQuaternionW();
    inputs.quaternion[1] = ahrs.getQuaternionX();
    inputs.quaternion[2] = ahrs.getQuaternionY();
    inputs.quaternion[3] = ahrs.getQuaternionZ();
  }
}
