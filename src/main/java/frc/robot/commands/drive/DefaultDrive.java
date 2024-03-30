package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultDrive extends Command {
  private final DrivebaseSubsystem drivebaseSubsystem;
  private final DoubleSupplier xJoystickDoubleSupplier;
  private final DoubleSupplier yJoystickDoubleSupplier;
  private final DoubleSupplier thetaJoystickDoubleSupplier;
  private final DoubleSupplier sclaerInputDoubleSupplier;
  private final BooleanSupplier feildOriented;

  private final SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(40);
  private final SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(40);
  private final SlewRateLimiter slewRateLimiterTheta = new SlewRateLimiter(30);
  private boolean sticky;

  public DefaultDrive(
      DoubleSupplier directionX,
      DoubleSupplier directionY,
      DoubleSupplier rotation,
      DoubleSupplier scalar,
      BooleanSupplier feildOriented,
      DrivebaseSubsystem drivebaseSubsystem) {
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.xJoystickDoubleSupplier = directionX;
    this.yJoystickDoubleSupplier = directionY;
    this.thetaJoystickDoubleSupplier = rotation;
    this.sclaerInputDoubleSupplier = scalar;
    this.feildOriented = feildOriented;

    addRequirements(drivebaseSubsystem);
  }

  @Override
  public void execute() {

    Double joyStickX =
        (MathUtil.applyDeadband(xJoystickDoubleSupplier.getAsDouble(), 0.1)
                * Constants.Drivebase.MAX_LINEAR_SPEED)
            * MathUtil.clamp(sclaerInputDoubleSupplier.getAsDouble() - 1, -1, -0.25)
            * 1;

    Double joyStickY =
        (MathUtil.applyDeadband(yJoystickDoubleSupplier.getAsDouble(), 0.05)
                * Constants.Drivebase.MAX_LINEAR_SPEED)
            * MathUtil.clamp(sclaerInputDoubleSupplier.getAsDouble() - 1, -1, -0.25)
            * 1;

    Double joyStickTheta =
        ((MathUtil.applyDeadband(thetaJoystickDoubleSupplier.getAsDouble(), 0.1)
                    * Constants.Drivebase.MAX_ANGULAR_SPEED)
                * MathUtil.clamp(sclaerInputDoubleSupplier.getAsDouble() - 1, -1, -0.25))
            * Constants.HID.thetaJoystickScalar
            * 1;

    // if (joyStickX != 0 || joyStickY != 0 || joyStickTheta != 0) {
    //   drivebaseSubsystem.drivejoysticks(
    //       new ChassisSpeeds(
    //           slewRateLimiterX.calculate(joyStickX),
    //           slewRateLimiterY.calculate(joyStickY),
    //           slewRateLimiterTheta.calculate(joyStickTheta)),
    //       feildOriented.getAsBoolean());
    //   sticky = false;
    // } else {
    //   if (!sticky) {
    //     drivebaseSubsystem.stopWithX();
    //     sticky = true;
    //   }
    // }
    drivebaseSubsystem.drivejoysticks(
        new ChassisSpeeds(
            slewRateLimiterX.calculate(joyStickX),
            slewRateLimiterY.calculate(joyStickY),
            slewRateLimiterTheta.calculate(joyStickTheta)),
        feildOriented.getAsBoolean());
  }
}
