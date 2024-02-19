package frc.robot.util.States;

public enum IntakeState {
  HOLDING, // The intake is currently holding a piece
  EMPTY, // The intake is currently empty
  INTAKING, // The intake is currently collecting a note
  TRANSFERING, // Transfering note from the intake to the shooter
  UNDEFINED; // The state is undefiend
}
