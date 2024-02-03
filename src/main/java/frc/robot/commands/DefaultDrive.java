package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import java.util.function.DoubleSupplier;

public class DefaultDrive extends Command {
  private final DrivebaseSubsystem drivebaseSubsystem;
  private final DoubleSupplier directionX;
  private final DoubleSupplier directionY;
  private final DoubleSupplier rotation;

  public DefaultDrive(
      DoubleSupplier directionX,
      DoubleSupplier directionY,
      DoubleSupplier rotation,
      DrivebaseSubsystem drivebaseSubsystem) {
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.directionX = directionX;
    this.directionY = directionY;
    this.rotation = rotation;

    addRequirements(drivebaseSubsystem);
  }

  @Override
  public void execute() {
    drivebaseSubsystem.runVelocity(
        new ChassisSpeeds(
            -MathUtil.applyDeadband(directionX.getAsDouble(), 0.1),
            -MathUtil.applyDeadband(directionY.getAsDouble(), 0.1),
            -MathUtil.applyDeadband(rotation.getAsDouble(), 0.1)));
  }
}
