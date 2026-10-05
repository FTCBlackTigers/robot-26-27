package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Runs the intake inwards until a game piece is inside, or until interrupted (e.g. the button
 * bound with {@code whenHeld} is released). Always stops the intake at the end.
 * <p>
 * Intake is still a placeholder, so for now this never detects a piece and nothing spins.
 */
public class IntakeIn extends CommandBase {
    private final Intake intake;

    public IntakeIn(Intake intake) {
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
