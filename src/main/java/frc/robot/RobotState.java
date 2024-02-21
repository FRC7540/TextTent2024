// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.States.FlywheelState;
import frc.robot.util.States.RobotNoteState;
import frc.robot.util.States.ShooterState;

public class RobotState {
  public static Pose2d robotPose2D;
  public static Pose3d robotPose3D;
  public static ShooterState shooterState;
  public static FlywheelState flywheelState;
  public static RobotNoteState robotNoteState;

  public RobotState() {
    shooterState = ShooterState.UNDEFINED;
    flywheelState = FlywheelState.UNDEFINED;
    robotNoteState = RobotNoteState.UNDEFINED;
  }
}
