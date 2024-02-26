package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shooting.IntakeToShooterFromIntake;
import frc.robot.commands.intake.TransferNoteToShooter;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class TransferNoteFromIntakeToShooter extends SequentialCommandGroup {
  public TransferNoteFromIntakeToShooter(
      IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    addCommands(
        new TransferNoteToShooter(intakeSubsystem),
        new IntakeToShooterFromIntake(shooterSubsystem));
  }
}
