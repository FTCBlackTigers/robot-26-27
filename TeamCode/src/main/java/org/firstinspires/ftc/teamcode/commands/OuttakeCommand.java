package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Runs the intake outwards (spit out / unjam) until interrupted. Bind it with {@code whenHeld}, or
 * add {@code .withTimeout(ms)} in Autonomous, since it never ends by itself.
 */
public class OuttakeCommand extends CommandBase {
    private final IntakeSubsystem intake;

    public OuttakeCommand(IntakeSubsystem intake) {
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
