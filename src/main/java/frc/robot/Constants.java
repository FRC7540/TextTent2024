// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Default feature flag settings, should only affect anythign if not intilaized.
  // WARNING: THIS CLASS IS REFLECTED! DO NOT PUT ANYTHING BUT BOOLEANS!
  // The varible name will be the name of the prefrence key.
  public static final class Flags {
    public static final boolean USE_PATH_PLANNER = true;
    public static final boolean USE_REAL_FLYWHEEL_HARDWARE = false;
  }

  public static final class HID {
    public static final int operatorControlerPort = 1;
    public static final int driverControllerPort = 0;
    public static final double thetaJoystickScalar = 1;
  }

  public static final class Drivebase {
    public static final double WHEEL_RADIUS = Units.inchesToMeters(1.5);
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
    public static final double WHEEL_CIRCUMFRENCE = WHEEL_DIAMETER * Math.PI;
    public static final int DRIVE_PINON = 13;
    public static final double TURN_GEAR_RATIO = 46.42;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double DRIVE_GEAR_RATIO = (45.0 * 22.0) / (DRIVE_PINON * 15.0);
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.63);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(29.0);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(29.0);
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    public static final double DRIVE_MOMENT_INERTIA = 0.025;
    public static final double TURN_MOMENT_INERTIA = 0.004;
    public static final double NOMINAL_LOOP_PERIOD = 0.02;

    public static final PathConstraints DEFAULT_PATHFINDING_CONSTRAINTS =
        new PathConstraints(2.0, 1.0, Units.degreesToRadians(100), Units.degreesToRadians(100));

    public static final class ModFL {
      public static final Rotation2d ABSOULUTE_OFFSET = new Rotation2d(-0.5 * Math.PI);
      public static final int DRIVE_SPARKMAX_CAN_ID = 23;
      public static final int TURN_SPARKMAX_CAN_ID = 22;
      public static final boolean TURN_MOTOR_INVERT = false;
    }

    public static final class ModFR {
      public static final Rotation2d ABSOULUTE_OFFSET = new Rotation2d(1.0 * Math.PI);
      public static final int DRIVE_SPARKMAX_CAN_ID = 21;
      public static final int TURN_SPARKMAX_CAN_ID = 20;
      public static final boolean TURN_MOTOR_INVERT = false;
    }

    public static final class ModBL {
      public static final Rotation2d ABSOULUTE_OFFSET = new Rotation2d(2.0 * Math.PI);
      public static final int DRIVE_SPARKMAX_CAN_ID = 27;
      public static final int TURN_SPARKMAX_CAN_ID = 26;
      public static final boolean TURN_MOTOR_INVERT = false;
    }

    public static final class ModBR {
      public static final Rotation2d ABSOULUTE_OFFSET = new Rotation2d(0.5 * Math.PI);
      public static final int DRIVE_SPARKMAX_CAN_ID = 25;
      public static final int TURN_SPARKMAX_CAN_ID = 24;
      public static final boolean TURN_MOTOR_INVERT = false;
    }

    public static final Translation2d[] MODULE_TRANSLATIONS = {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    }; // FL, FR, BL, BR
  }

  public static final class Shooter {
    public static final double NOMINAL_LOOP_PERIOD = 0.02;
    public static final int PUSHER_MOTOR_CAN_ID = 12;
    public static final MotorType SPARK_MAX_MOTOR_ONE_TYPE = MotorType.kBrushed;
    public static final boolean PUSHER_MOTOR_INVERTED = false;

    public static final int HOLDER_LIMIT_SWITCH_PORT = 1;
    public static final int SHOT_LIMIT_SWITCH_PORT = 2;

    public static final double FLYWHEEL_SPEED_THRESHOLD = 2;

    public static final double SHOOTER_ANGLE_RADIANS = 1;

    public static final double NOTE_MAXIMUM_HEIGHT_METERS = 69;

    public static enum Direction {
      FORWARD(6.0),
      STOP(0.0),
      BACK(-6.0);
      private double voltage;

      Direction(double voltage) {
        this.voltage = voltage;
      }

      public double getVoltage() {
        return voltage;
      }
    }

    public static final double PUSHER_WAIT_TIME = 2.0;
    public static final double PUSHER_MOMENT_OF_INTERTIA = 0.001;
    public static final DCMotor PUSHER_MOTOR_TYPE = DCMotor.getNeo550(1);
    public static final double PUSHER_MOTOR_GEART_RATIO = 1.0;

    public static final class Flywheel {
      public static final int MOTOR_ONE_CAN_ID = 10;
      public static final int MOTOR_TWO_CAN_ID = 11;
      public static final double SIM_UPDATE_TIME = 0.02;

      // qelms. Velocity error tolerance, in radians per second. Decrease
      // this to more heavily penalize state excursion, or make the controller behave more
      // aggressively.
      // relms. Control effort (voltage) tolerance. Decrease this to more
      // heavily penalize control effort, or make the controller less aggressive. 12 is a good
      // starting point because that is the (approximate) maximum voltage of a battery.
      public static final class WheelOne {
        public static final Matrix<N1, N1> MODEL_STD_DEV = VecBuilder.fill(3.0);
        public static final Matrix<N1, N1> MEASUREMENT_STD_DEV = VecBuilder.fill(0.01);
        public static final double NOMINAL_DISCRETIZATION_TIMESTEP = 0.020; // Seconds
        public static final Vector<N1> QELMS = VecBuilder.fill(16.0); // Rads per second
        public static final Vector<N1> RELMS = VecBuilder.fill(12.0); // Volts
        public static final double MOMENT_OF_INERTIA = 0.11;
        public static final int MOTOR_COUNT = 1;
        public static final double GEAR_RATIO = 4.0;
        public static final boolean INVERTED = true;
        public static final double MAX_VOLTAGE = 12.0; // Volts
      }

      public static final class WheelTwo {
        public static final Matrix<N1, N1> MODEL_STD_DEV = VecBuilder.fill(3.0);
        public static final Matrix<N1, N1> MEASUREMENT_STD_DEV = VecBuilder.fill(0.01);
        public static final double NOMINAL_DISCRETIZATION_TIMESTEP = 0.020; // Seconds
        public static final Vector<N1> QELMS = VecBuilder.fill(16.0); // Rads per second
        public static final Vector<N1> RELMS = VecBuilder.fill(12.0); // Volts
        public static final double MOMENT_OF_INERTIA = 0.11;
        public static final int MOTOR_COUNT = 1;
        public static final double GEAR_RATIO = 4.0;
        public static final boolean INVERTED = false;
        public static final double MAX_VOLTAGE = 12.0; // Volts
      }
    }
  }

  public static final class Intake {
    public static final int MOTOR_ONE_CAN_ID = 13;
    public static final MotorType SPARK_MOTOR_ONE_TYPE = MotorType.kBrushless;
    public static final boolean MOTOR_ONE_INVERTED = false;
    public static final int INTAKE_NOTE_LIMIT_SWITCH_PORT = 0;
    public static final double MOTOR_MOMENT_OF_INTERTIA = 0.001;
    public static final DCMotor MOTOR_TYPE = DCMotor.getNeo550(1);
    public static final double MOTOR_GEAR_RATIO = 1.0;
    public static final double INTAKE_TIME_SECONDS = 1.5;
  }

  public static final class Climber {
    public static final double EXTENSION_RADIANS = 0.0;
    public static final double EXTENSION_THRESHOLD = 0.0;
    public static final double GEAR_RATIO = 1.0;
    public static final double SPOOL_SIZE_METERS = 1.0;
    public static final int CLIMBER_MOTOR_CONTROLLER_CAN_ID = 16;
    public static final int CLIMBER_ENCODER_PORT = 4;
    public static final double CLIMBER_ENCODER_DISTANCE_PER_ROTATION = 1;
  }

  public static final class Vision {
    public static final String APRIL_TAG_NETWORKTABLE_TOPIC_NAME = "limelight-april";
  }

  public static final class Field {
    public static final class Blue {
      public static final Pose2d SPEAKER_POSE2D = new Pose2d(-0.0381, 5.547868, new Rotation2d());
    }

    public static final class Red {
      public static final Pose2d SPEAKER_POSE2D = new Pose2d(16.579342, 5.547868, new Rotation2d());
    }
  }

  public enum Targets {
    SPEAKER,
    AMP,
    OURSIDE;
  }
}
