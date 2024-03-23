package frc.robot.util.types;

import edu.wpi.first.math.geometry.Rotation2d;

public record TargetNote(
    Rotation2d xError, Rotation2d yError, double targetArea, String targetClass) {}
