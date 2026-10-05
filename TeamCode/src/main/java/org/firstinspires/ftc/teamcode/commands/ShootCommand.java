package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * Shoots one game piece: spin up (gives up waiting after {@link #SPIN_UP_TIMEOUT_MS}), push the piece
 * in with the feeder, wait, retract the feeder. The flywheel keeps spinning for the next shot; stop it
 * with {@code shooter.stop()}.
 * <p>
 * The feeder is always retracted at the end, even if the command is interrupted halfway.
 */
public class ShootCommand extends SequentialCommandGroup {
    // Tune once the shooter exists
    public static long SPIN_UP_TIMEOUT_MS = 1500;
    public static long FEED_TIME_MS = 300;

    private final ShooterSubsystem shooter;

    public ShootCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addCommands(
                new SpinUpShooterCommand(shooter).withTimeout(SPIN_UP_TIMEOUT_MS),
                new InstantCommand(shooter::feed, shooter),
                new WaitCommand(FEED_TIME_MS),
                new InstantCommand(shooter::retractFeeder, shooter)
        );
        addRequirements(shooter);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shooter.retractFeeder();
    }
}
