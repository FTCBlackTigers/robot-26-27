package org.firstinspires.ftc.teamcode.commands.drive;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

/**
 * Corrects where the robot thinks it is (x, y) from the AprilTags, keeping the Pinpoint heading
 * (much more exact than the camera's). Odometry drifts over an auto; this snaps it back.
 * Use it after a path, while the robot is stopped and sees tags, e.g. right before shooting:
 * {@code drive(toShootingSpot()), new RelocalizeFromCamera(robot.drivetrain, robot.limelight), ...}
 * <p>
 * Safe to leave in an auto: it averages {@link #SAMPLES} readings taken while stopped, ignores readings
 * more than {@link #MAX_CORRECTION_IN} away from odometry (a misread tag), and after
 * {@link #TIMEOUT_MS} without enough readings it ends and changes nothing.
 * <p>
 * Doesn't drive, so it doesn't require the drivetrain. After a FollowPathCommand Pedro is still
 * holding the path's end point, so once the pose is corrected the robot drives itself onto the real spot.
 */
public class RelocalizeFromCamera extends CommandBase {
    public static int SAMPLES = 5;                 // readings to average (one per loop)
    public static double MAX_CORRECTION_IN = 24;   // farther than this from odometry = misread, ignored
    public static double TIMEOUT_MS = 1000;        // give up after this, without changing the pose

    private final Drivetrain drivetrain;
    private final Limelight limelight;
    private final ElapsedTime timer = new ElapsedTime();
    private double sumX, sumY;
    private int count;

    public RelocalizeFromCamera(Drivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        timer.reset();
        sumX = 0;
        sumY = 0;
        count = 0;
    }

    @Override
    public void execute() {
        // Only while stopped: the camera's picture is a few ms old, so a moving robot reads behind
        Pose camera = limelight.getPedroPose();
        if (camera == null || drivetrain.isMoving()) return;
        if (camera.distance(drivetrain.getPose()) > MAX_CORRECTION_IN) return;

        sumX += camera.x();
        sumY += camera.y();
        count++;
    }

    @Override
    public boolean isFinished() {
        return count >= SAMPLES || timer.milliseconds() > TIMEOUT_MS;
    }

    @Override
    public void end(boolean interrupted) {
        if (count < SAMPLES) return; // timed out or interrupted: keep the odometry pose
        drivetrain.setPose(new Pose(sumX / count, sumY / count, drivetrain.getPose().heading()));
    }
}
