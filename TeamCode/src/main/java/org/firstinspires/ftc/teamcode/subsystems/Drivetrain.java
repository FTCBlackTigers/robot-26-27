package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Drivetrain, backed by the Pedro Pathing {@link Follower}.
 * <p>
 * TeleOp and Autonomous share this one drivetrain: the same motors, odometry (Pinpoint) and pose.
 * Motor names, directions and brake mode live in {@link Constants}. Autonomous passes
 * {@link #getFollower()} to the SolversLib Pedro commands (FollowPathCommand, TurnCommand, ...).
 * <p>
 * Units: inches, degrees, inches per second. Position and heading come from the Pinpoint, so there are
 * no drive-motor encoders to reset: use {@link #setPose} / {@link #resetHeading} / {@link #resetDistance}.
 * <p>
 * {@link #periodic()} calls {@code follower.update()} every loop, so OpModes must NOT also call it.
 */
public class Drivetrain extends SubsystemBase {
    /** Below this speed (inches/s) and turn rate (degrees/s) the robot counts as stopped. */
    public static double MOVING_SPEED_THRESHOLD = 1.0;
    public static double MOVING_TURN_THRESHOLD = 5.0;

    private final Follower follower;
    private final Telemetry telemetry;

    private double speedMultiplier = 1.0;
    private double distanceTraveled = 0;
    private Pose lastPose;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        follower = Constants.createFollower(hardwareMap);
        lastPose = follower.pose();
    }

    // ------------ Driving ------------

    /**
     * Field-centric drive: pushing "forward" always moves away from the driver, whatever way the robot
     * faces. Inputs are -1..1 in Pedro's frame: +forward, +left, +counterclockwise
     * (DriveWithController converts the gamepad sticks). Scaled by {@link #setSpeedMultiplier}.
     */
    public void driveFieldCentric(double forward, double left, double turn) {
        follower.manual(ManualDrive.fieldCentric(
                forward * speedMultiplier,
                left * speedMultiplier,
                turn * speedMultiplier,
                follower.pose().heading()));
    }

    /** Robot-centric drive ("forward" = where the robot faces), same inputs as {@link #driveFieldCentric}. */
    public void driveRobotCentric(double forward, double left, double turn) {
        follower.manual(forward * speedMultiplier, left * speedMultiplier, turn * speedMultiplier);
    }

    public void stop() {
        follower.manual(0, 0, 0);
    }

    /**
     * Max speed for manual driving, 0..1 (1 = full speed, 0.4 = slow mode). This is setMULT in the spec.
     * Doesn't affect autonomous paths, which have their own max power.
     */
    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = Math.max(0, Math.min(1, multiplier));
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    // ------------ Heading and position ------------

    /** Current heading in degrees (0 = the direction set by the last reset). */
    public double getHeading() {
        return Math.toDegrees(follower.pose().heading());
    }

    /** Makes the robot's current facing the new 0 (the new "forward" for field-centric driving). */
    public void resetHeading() {
        Pose pose = follower.pose();
        setPose(new Pose(pose.x(), pose.y(), 0));
    }

    /** Position on the field: x, y in inches, heading in radians. */
    public Pose getPose() {
        return follower.pose();
    }

    /** Tells the robot where it is (e.g. the start position of an auto). */
    public void setPose(Pose pose) {
        follower.setPose(pose);
        lastPose = pose; // a teleport isn't distance traveled
    }

    // ------------ Distance ------------

    /** Total distance driven in inches since the last {@link #resetDistance()} (like a car's trip meter). */
    public double getDistanceTraveled() {
        return distanceTraveled;
    }

    public void resetDistance() {
        distanceTraveled = 0;
    }

    // ------------ Status ------------

    /** Driving speed in inches per second (any direction). */
    public double getSpeed() {
        Velocity velocity = follower.velocity();
        return Math.hypot(velocity.vx, velocity.vy);
    }

    /** Turning speed in degrees per second (+ = counterclockwise). */
    public double getTurnSpeed() {
        return Math.toDegrees(follower.velocity().omega);
    }

    /** True while the robot is driving or turning. */
    public boolean isMoving() {
        return getSpeed() > MOVING_SPEED_THRESHOLD || Math.abs(getTurnSpeed()) > MOVING_TURN_THRESHOLD;
    }

    /** True while an autonomous path / turn / hold is running. */
    public boolean isBusy() {
        return follower.isBusy();
    }

    /** True if the motors brake when you let go of the sticks (set in Constants: manualBrakeMode). */
    public boolean isBrakeMode() {
        return Constants.drivetrainConfig.manualBrakeMode.get();
    }

    public Follower getFollower() {
        return follower;
    }

    @Override
    public void periodic() {
        follower.update();

        Pose pose = follower.pose();
        distanceTraveled += pose.distance(lastPose);
        lastPose = pose;

        telemetry.addData("Position", "x %.1f  y %.1f  heading %.1f°", pose.x(), pose.y(), getHeading());
        telemetry.addData("Speed", "%.1f in/s  turn %.0f°/s%s", getSpeed(), getTurnSpeed(), isMoving() ? "" : "  (stopped)");
        telemetry.addData("Distance", "%.1f in", distanceTraveled);
        telemetry.addData("Speed multiplier", "%.2f", speedMultiplier);
    }
}
