package org.firstinspires.ftc.teamcode.commands.shooter;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Pushes one game piece into the flywheel, holds for {@link #FEED_TIME_MS}, then retracts.
 * The feeder is retracted at the end even if this is interrupted.
 */
public class Feed extends CommandBase {
    // Tune once the shooter exists
    public static long FEED_TIME_MS = 300;

    private final Shooter shooter;
    private final ElapsedTime timer = new ElapsedTime();

    public Feed(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.feed();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= FEED_TIME_MS;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.retractFeeder();
    }
}
