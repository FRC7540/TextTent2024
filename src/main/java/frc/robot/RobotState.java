// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.States.IntakeState;

public class RobotState {
  private Pose2d robotPose2D;
  private Pose3d robotPose3D;
  private IntakeState intakeStates;

  public RobotState() {}

  /*Set the robots 2d pose to this position, does not change the 3d pose
   */
  public void setRobotPose(Pose2d pose) {
    this.robotPose2D = pose;
  }

  public void setRobotPose(Pose3d pose) {
    this.robotPose3D = pose;
    this.robotPose2D = pose.toPose2d();
  }

  /* Gets the current authoratative 2d pose of the robot */
  public Pose2d getCurrentTobotPose2d() {
    return robotPose2D;
  }

  /* Gets the current authoratative 3d pose of the robot */
  public Pose3d getCurrentTobotPose3d() {
    return robotPose3D;
  }
}
