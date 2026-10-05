package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Runs the intake outwards (spit out / unjam) until interrupted. Bind it with {@code whenHeld}, or
 * add {@code .withTimeout(ms)} in Autonomous, since it never ends by itself.
 */
public class IntakeOut extends CommandBase {
    private final Intake intake;

    public IntakeOut(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.outtake();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
