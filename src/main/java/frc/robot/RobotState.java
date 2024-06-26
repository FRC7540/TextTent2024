// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.States.ClimberState;
import frc.robot.util.States.IntakeState;
import frc.robot.util.States.NoiseCancelingState;
import frc.robot.util.States.RobotNoteState;
import frc.robot.util.States.ShooterState;
import frc.robot.util.types.TargetNote;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  public static Pose2d robotPose2D;
  public static Pose3d robotPose3D;
  public static ShooterState shooterState;
  public static IntakeState intakeState;
  public static ClimberState climberState;
  public static RobotNoteState robotNoteState;
  public static Pose2d noteTargetPose;
  public static Pose3d botVisionPose;
  public static TargetNote targetNote;
  public static NoiseCancelingState noiseCancelingState;

  static {
    botVisionPose = new Pose3d();
    robotPose2D = new Pose2d();
    robotPose3D = new Pose3d();
    shooterState = ShooterState.UNDEFINED;
    intakeState = IntakeState.UNDEFINED;
    climberState = ClimberState.UNDEFINED;
    robotNoteState = RobotNoteState.UNDEFINED;
    noteTargetPose = new Pose2d();
    targetNote = new TargetNote(new Rotation2d(), new Rotation2d(), 0, "");
    noiseCancelingState = NoiseCancelingState.UNDEFINED;
  }

  public static void pushUpdate() {
    Logger.recordOutput("RobotState/robotPose2D", robotPose2D);
    Logger.recordOutput("RobotState/robotPose3D", robotPose3D);
    Logger.recordOutput("RobotState/shooterState", shooterState);
    Logger.recordOutput("RobotState/intakeState", intakeState);
    Logger.recordOutput("RobotState/climberState", climberState);
    Logger.recordOutput("RobotState/robotNoteState", robotNoteState);
    Logger.recordOutput("RobotState/noteTargetPose", noteTargetPose);
    Logger.recordOutput("RobotState/targetNote", targetNote.toString());
    Logger.recordOutput("RobotState/noiseCancelingState", noiseCancelingState);
  }
}
