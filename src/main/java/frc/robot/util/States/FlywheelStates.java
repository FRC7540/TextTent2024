package frc.robot.util.States;

public enum FlywheelStates {
  STOPPED, // Flywheels are stopped
  SPINNING_UP, // Flywheels are spinning to desired velocity
  SPINNING_DOWN, // Flywheels are slowing down to stop
  AT_SPEED, // Flywheels are at speed and ready to shoot
  UNDEFINED; // Something is wrong
}
