package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import frc.robot.util.types.TargetNote;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveLockedToNote extends Command {
  private final Supplier<TargetNote> targetNoteSupplier;
  private final DrivebaseSubsystem drivebaseSubsystem;
  private DoubleSupplier xJoystickDoubleSupplier;
  private DoubleSupplier yJoystickDoubleSupplier;
  private DoubleSupplier scalar;
  private final PIDController pidController;

  private final SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(40);
  private final SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(40);

  public DriveLockedToNote(
      Supplier<TargetNote> targetNoteSupplier,
      DoubleSupplier xJoystickDoubleSupplier,
      DoubleSupplier yJoystickDoubleSupplier,
      DoubleSupplier scalarInputDoubleSupplier,
      DrivebaseSubsystem drivebaseSubsystem) {
    this.targetNoteSupplier = targetNoteSupplier;
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.xJoystickDoubleSupplier = xJoystickDoubleSupplier;
    this.yJoystickDoubleSupplier = yJoystickDoubleSupplier;
    this.scalar = scalarInputDoubleSupplier;

    pidController = new PIDController(2, 0.2, 0);
    pidController.setSetpoint(new Rotation2d().getRadians());
    // May not want this, should test
    // pidController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drivebaseSubsystem);
  }

  @Override
  public void execute() {
    drivebaseSubsystem.drivejoysticks((calculateChassisSpeeds()), false);
  }

  public ChassisSpeeds calculateChassisSpeeds() {
    return new ChassisSpeeds(
        slewRateLimiterX.calculate(getJoystickXClean()),
        slewRateLimiterY.calculate(getJoystickYClean()),
        pidController.calculate(targetNoteSupplier.get().xError().getRadians() * -1));
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
