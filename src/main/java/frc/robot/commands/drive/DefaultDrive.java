package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class DefaultDrive extends Command {
  private final DrivebaseSubsystem drivebaseSubsystem;
  private final DoubleSupplier directionX;
  private final DoubleSupplier directionY;
  private final DoubleSupplier rotation;
  private final DoubleSupplier scalar;
  private final LoggedDashboardBoolean feildOriented;

  private final SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(1);
  private final SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(1);
  private final SlewRateLimiter slewRateLimiterTheta = new SlewRateLimiter(1);

  public DefaultDrive(
      DoubleSupplier directionX,
      DoubleSupplier directionY,
      DoubleSupplier rotation,
      DoubleSupplier scalar,
      DrivebaseSubsystem drivebaseSubsystem) {
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.directionX = directionX;
    this.directionY = directionY;
    this.rotation = rotation;
    this.scalar = scalar;

    feildOriented = new LoggedDashboardBoolean("feild Oriented", true);
    addRequirements(drivebaseSubsystem);
  }

  @Override
  public void execute() {

    Double joyStickX =
        (MathUtil.applyDeadband(directionX.getAsDouble(), 0.1)
                * Constants.Drivebase.MAX_LINEAR_SPEED)
            * (scalar.getAsDouble() - 1)
            * 1.0;

    Double joyStickY =
        (MathUtil.applyDeadband(directionY.getAsDouble(), 0.1)
                * Constants.Drivebase.MAX_LINEAR_SPEED)
            * (scalar.getAsDouble() - 1)
            * 1.0;

    Double joyStickTheta =
        ((MathUtil.applyDeadband(rotation.getAsDouble(), 0.1)
                    * Constants.Drivebase.MAX_ANGULAR_SPEED)
                * (scalar.getAsDouble() - 1))
            * Constants.HID.thetaJoystickScalar
            * 1.0;

    drivebaseSubsystem.drivejoysticks(
        new ChassisSpeeds(
            slewRateLimiterX.calculate(joyStickX),
            slewRateLimiterY.calculate(joyStickY),
            slewRateLimiterTheta.calculate(joyStickTheta)),
        feildOriented.get());
  }
}
