package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;

public class PathFindToPoint extends Command {
  private Command pathfindingCommand;

  public PathFindToPoint(Pose2d targetPose, DrivebaseSubsystem drivebaseSubsystem) {

    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    pathfindingCommand =
        AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before
            // attempting to rotate.
            );
    addRequirements(drivebaseSubsystem);
  }

  @Override
  public void initialize() {
    pathfindingCommand.schedule();
  }

  @Override
  public void cancel() {
    pathfindingCommand.cancel();
  }

  @Override
  public boolean isFinished() {
    return pathfindingCommand.isFinished();
  }
}
