package frc.robot.util.States;

public enum ShooterState {
  EMPTY, // The robot has no note and is ready to intake
  LOADING, // The robot is intaking a piece
  LOADED, // The robot has a note in the "chamber" and can be spun up
  ARMING, // The robot is arming
  ARMED, // The robot is armed and can fire at any moment
  SHOOTING, // The robot is activley shooting a piece, this shouldnt last long
  RECOVERING, // The robot is recovering from a shot
  UNDEFINED; // We just shot, spinning down;
}
