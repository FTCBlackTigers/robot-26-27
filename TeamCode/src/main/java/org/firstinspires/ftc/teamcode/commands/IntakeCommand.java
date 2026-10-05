package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Runs the intake inwards until a game piece is inside, or until interrupted (e.g. the button
 * bound with {@code whenHeld} is released). Always stops the intake at the end.
 * <p>
 * IntakeSubsystem is still a placeholder, so for now this never detects a piece and nothing spins.
 */
public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;

    public IntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intake();
    }

    @Override
    public boolean isFinished() {
        return intake.hasGamePiece();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
