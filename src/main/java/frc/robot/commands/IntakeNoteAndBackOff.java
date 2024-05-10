package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class IntakeNoteAndBackOff extends SequentialCommandGroup {

  public IntakeNoteAndBackOff(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    addCommands(
        new IntakeNote(intakeSubsystem, shooterSubsystem),
        new RunCommand(() -> shooterSubsystem.setPusherVoltage(-1.5))
            .withTimeout(0.2)
            .andThen(new InstantCommand(() -> shooterSubsystem.setPusherVoltage(0))));
  }
}
