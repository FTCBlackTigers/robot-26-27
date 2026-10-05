package org.firstinspires.ftc.teamcode.commands.drive;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Makes the direction the robot faces right now "forward" for field-centric driving. Ends
 * immediately. Doesn't require the drivetrain, so driving isn't interrupted.
 */
public class ResetHeading extends CommandBase {
    private final Drivetrain drivetrain;

    public ResetHeading(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        drivetrain.resetHeading();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
