package org.firstinspires.ftc.teamcode.commands.drive;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.function.DoubleConsumer;

/**
 * Turns toward the Limelight target until it's centered (tx = 0), and keeps it centered while the
 * button is held. With no target in view it stops and waits.
 * <p>
 * Right now the whole robot turns. Later a turret can use the exact same aiming math: give it the
 * turret instead of the drivetrain, e.g. {@code new AimAtTarget(limelight, turret::setPower, turret)}.
 * The turn power passed in is -1..1, + = counterclockwise (seen from above).
 * <p>
 * Tuning (on the robot, with Limelight Test open to see tx):
 * <ul>
 *     <li>Turns the wrong way and spins away: tx sign is flipped, change {@code -tx} to {@code tx}.</li>
 *     <li>Too slow / stops short of center: raise {@link #MIN_POWER}, then {@link #KP}.</li>
 *     <li>Overshoots and wiggles around the target: lower {@link #KP}.</li>
 * </ul>
 */
public class AimAtTarget extends CommandBase {
    public static double KP = 0.02;           // turn power per degree of tx
    public static double MIN_POWER = 0.08;    // smallest power that still actually turns the robot
    public static double MAX_POWER = 0.5;     // safety cap
    public static double TOLERANCE_DEG = 1.0; // closer than this counts as aimed

    private final Limelight limelight;
    private final DoubleConsumer turn;

    /** Aim by turning the whole robot. */
    public AimAtTarget(Drivetrain drivetrain, Limelight limelight) {
        this(limelight, power -> drivetrain.driveRobotCentric(0, 0, power), drivetrain);
    }

    /** Aim with anything that rotates (drivetrain now, turret later). */
    public AimAtTarget(Limelight limelight, DoubleConsumer turn, Subsystem rotator) {
        this.limelight = limelight;
        this.turn = turn;
        // Only the rotating part is required: the Limelight is just read
        addRequirements(rotator);
    }

    @Override
    public void execute() {
        if (!limelight.hasTarget() || isAimed()) {
            turn.accept(0);
            return;
        }

        // tx is + when the target is to the right, and turning right is clockwise (negative)
        double power = -limelight.getTx() * KP;
        // A minimum power so small errors still move it, a cap so it never whips around
        double magnitude = Math.min(MAX_POWER, Math.max(MIN_POWER, Math.abs(power)));
        turn.accept(Math.copySign(magnitude, power));
    }

    /** True when the target is in view and centered within {@link #TOLERANCE_DEG}. */
    public boolean isAimed() {
        return limelight.hasTarget() && Math.abs(limelight.getTx()) < TOLERANCE_DEG;
    }

    // Never finishes by itself, so holding the button keeps the target centered.
    // In an auto, end it with e.g. .withTimeout(1500)

    @Override
    public void end(boolean interrupted) {
        turn.accept(0);
    }
}
