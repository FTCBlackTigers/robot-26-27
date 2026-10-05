package org.firstinspires.ftc.teamcode.commands.automations;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.shooter.Feed;
import org.firstinspires.ftc.teamcode.commands.shooter.SpinUp;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Shoots one game piece: spin up (gives up waiting after {@link #SPIN_UP_TIMEOUT_MS}), then feed.
 * The flywheel keeps spinning for the next shot; stop it with StopShooter.
 * <p>
 * Automations are sequences built from the single-subsystem commands in the other folders. When the
 * intake and shooter need to work together (e.g. intake indexes the next piece between shots), this
 * is where that goes.
 */
public class Shoot extends SequentialCommandGroup {
    // Tune once the shooter exists
    public static long SPIN_UP_TIMEOUT_MS = 1500;

    public Shoot(Shooter shooter) {
        addCommands(
                new SpinUp(shooter).withTimeout(SPIN_UP_TIMEOUT_MS),
                new Feed(shooter)
        );
    }
}
