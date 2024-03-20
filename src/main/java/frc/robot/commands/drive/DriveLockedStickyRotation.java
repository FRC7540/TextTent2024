package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveLockedStickyRotation extends Command {
  private final Supplier<Rotation2d> targetRotationSupplier;
  private final DrivebaseSubsystem drivebaseSubsystem;
  private DoubleSupplier xJoystickDoubleSupplier;
  private DoubleSupplier yJoystickDoubleSupplier;
  private DoubleSupplier scalar;
  private final PIDController pidController;

  private final SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(40);
  private final SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(40);

  private double setpoint = 0.0;

  public DriveLockedStickyRotation(
      Supplier<Rotation2d> targetRotationSupplier,
      DoubleSupplier xJoystickDoubleSupplier,
      DoubleSupplier yJoystickDoubleSupplier,
      DoubleSupplier scalarInputDoubleSupplier,
      DrivebaseSubsystem drivebaseSubsystem) {
    this.targetRotationSupplier = targetRotationSupplier;
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.xJoystickDoubleSupplier = xJoystickDoubleSupplier;
    this.yJoystickDoubleSupplier = yJoystickDoubleSupplier;
    this.scalar = scalarInputDoubleSupplier;

    pidController = new PIDController(0.5, 0, 0);
    pidController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drivebaseSubsystem);
  }

  @Override
  public void initialize() {
    setpoint = targetRotationSupplier.get().getRadians();
  }

  @Override
  public void execute() {
    drivebaseSubsystem.drivejoysticks((calculateChassisSpeeds()), true);
  }

  public ChassisSpeeds calculateChassisSpeeds() {
    return new ChassisSpeeds(
        slewRateLimiterX.calculate(getJoystickXClean()),
        slewRateLimiterY.calculate(getJoystickYClean()),
        pidController.calculate(drivebaseSubsystem.getRotationRadians(), setpoint));
  }

  private double getJoystickXClean() {
    return (MathUtil.applyDeadband(xJoystickDoubleSupplier.getAsDouble(), 0.1)
            * Constants.Drivebase.MAX_LINEAR_SPEED)
        * MathUtil.clamp(scalar.getAsDouble() - 1, -1, -0.25)
        * 1.0;
  }

  private double getJoystickYClean() {
    return (MathUtil.applyDeadband(yJoystickDoubleSupplier.getAsDouble(), 0.1)
            * Constants.Drivebase.MAX_LINEAR_SPEED)
        * MathUtil.clamp(scalar.getAsDouble() - 1, -1, -0.25)
        * 1.0;
  }
}
