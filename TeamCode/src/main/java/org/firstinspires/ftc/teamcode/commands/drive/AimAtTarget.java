package org.firstinspires.ftc.teamcode.commands.drive;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

/**
 * SHELL: turn the robot to face the Limelight target (e.g. the goal's AprilTag).
 * <p>
 * The logic is commented out until the robot and pipeline exist; for now it ends immediately so
 * binding it can't lock up the drivetrain. To finish it: uncomment, tune {@link #TURN_KP} and
 * {@link #TOLERANCE_DEG}, and delete the {@code return true} in {@link #isFinished()}.
 * It only requires the drivetrain: the Limelight is read-only, so other commands can still use it.
 */
public class AimAtTarget extends CommandBase {
    // Tune once the robot exists
    public static double TURN_KP = 0.02; // turn power per degree of error
    public static double TOLERANCE_DEG = 1.0;

    private final Drivetrain drivetrain;
    private final Limelight limelight;

    public AimAtTarget(Drivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // if (!limelight.hasTarget()) return;
        // // tx is positive when the target is to the right; Pedro turns counterclockwise for positive
        // double turn = -limelight.getTx() * TURN_KP;
        // drivetrain.driveRobotCentric(0, 0, turn);
    }

    @Override
    public boolean isFinished() {
        // return limelight.hasTarget() && Math.abs(limelight.getTx()) < TOLERANCE_DEG;
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // drivetrain.stop();
    }
}
