package frc.robot.util.States;

public enum RobotStates {
    EMPTY, // The robot has no note and is ready to intake
    READY_TO_ARM, // The robot has a note and is ready arm (spin up flywheel)
    READY_TO_SHOOT, // The robot is armed and ready to shoot (flywheels are at speed)
    RECOVERING, // We just shot, spinnign down
    INTAKING; // Intaking a note, or the note is in the intake
}
