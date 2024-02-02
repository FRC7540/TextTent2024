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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
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
    public static final boolean USEPATHPLANNER = true;
  }

  public static final class Drivebase {
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    public static final double TURN_GEAR_RATIO = 4;
    public static final double DRIVE_GEAR_RATIO = 4;
    public static final double DRIVE_MOMENT_INERTIA = 0.025;
    public static final double TURN_MOMENT_INERTIA = 0.004;
    public static final double NOMINAL_LOOP_PERIOD = 0.02;

    public static final class Mod0 {
      public static final Rotation2d ABSOULUTE_OFFSET = new Rotation2d(0.0);
      public static final int DRIVE_SPARKMAX_CAN_ID = 12;
      public static final int TURN_SPARKMAX_CAN_ID = 13;
      public static final boolean TURN_MOTOR_INVERT = false;
    }

    public static final class Mod1 {
      public static final Rotation2d ABSOULUTE_OFFSET = new Rotation2d(0.0);
      public static final int DRIVE_SPARKMAX_CAN_ID = 14;
      public static final int TURN_SPARKMAX_CAN_ID = 15;
      public static final boolean TURN_MOTOR_INVERT = false;
    }

    public static final class Mod2 {
      public static final Rotation2d ABSOULUTE_OFFSET = new Rotation2d(0.0);
      public static final int DRIVE_SPARKMAX_CAN_ID = 16;
      public static final int TURN_SPARKMAX_CAN_ID = 17;
      public static final boolean TURN_MOTOR_INVERT = false;
    }

    public static final class Mod3 {
      public static final Rotation2d ABSOULUTE_OFFSET = new Rotation2d(0.0);
      public static final int DRIVE_SPARKMAX_CAN_ID = 18;
      public static final int TURN_SPARKMAX_CAN_ID = 19;
      public static final boolean TURN_MOTOR_INVERT = false;
    }

    public static final Translation2d[] MODULE_TRANSLATIONS = {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    }; // FL, FR, BL, BR
  }

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
      public static final double MOMENT_OF_INERTIA = 0.0005;
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
      public static final double MOMENT_OF_INERTIA = 0.0005;
      public static final int MOTOR_COUNT = 1;
      public static final double GEAR_RATIO = 4.0;
      public static final boolean INVERTED = false;
      public static final double MAX_VOLTAGE = 12.0; // Volts
    }
  }
}
