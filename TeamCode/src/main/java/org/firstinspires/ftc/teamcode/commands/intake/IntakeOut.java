package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Runs the intake while the button is held (bound with whenHeld in Robot.java).
 * Copy this file as a template for new commands.
 */
public class IntakeOut extends CommandBase {
    private final Intake intake;

    public IntakeOut(Intake intake) {
        this.intake = intake;
        addRequirements(intake); // only one command can use the intake at a time
    }

    // Runs once when the command starts
    @Override
        // motor.setPower(1);
    public void initialize() {
        intake.outtake();
    }

    // Runs once when the command ends or is interrupted (button released)
    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
