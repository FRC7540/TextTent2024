package frc.robot.util.States;

public enum FlywheelState {
  STOPPED, // Flywheels are stopped
  SPINNING_UP, // Flywheels are spinning to desired velocity
  SPINNING_DOWN, // Flywheels are slowing down to stop
  AT_SPEED, // Flywheels are at speed and ready to shoot
  UNDEFINED; // Flywheels are in a undefined state
}
