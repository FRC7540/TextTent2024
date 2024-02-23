package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;

public class Functions {

  public static Rotation2d rotationFromPoseToTarget(Pose2d targetPose, Pose2d botPose) {
    return new Rotation2d(
        Math.atan2(targetPose.getY() - botPose.getY(), targetPose.getX() - botPose.getX()));
  }

  public static Rotation2d rotationFromPoseToTarget(
      Supplier<Pose2d> targetPose, Supplier<Pose2d> botPose) {
    return new Rotation2d(
        Math.atan2(
            targetPose.get().getY() - botPose.get().getY(),
            targetPose.get().getX() - botPose.get().getX()));
  }
}
