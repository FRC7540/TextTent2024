package frc.robot.util.States;

public enum ShooterState {
  EMPTY("Empty"), // The robot has no note and is ready to intake
  LOADING("Loading"), // The robot is intaking a piece
  LOADED("Loaded"), // The robot has a note in the "chamber" and can be spun up
  ARMING("Arming"), // The robot is arming
  ARMED("Armed"), // The robot is armed and can fire at any moment
  SHOOTING("Shooting"), // The robot is activley shooting a piece, this shouldnt last long
  RECOVERING("Recovering"), // The robot is recovering from a shot
  SAFING("Safing"), // The robot is still loaded but we are spinning the flywheels down
  UNDEFINED("Undefined"); // We just shot, spinning down;

  private final String key;

  ShooterState(String key) {
    this.key = key;
  }

  public String toString() {
    return key;
  }
}
