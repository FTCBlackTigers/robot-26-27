package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Drivetrain, backed by the Pedro Pathing {@link Follower}.
 * <p>
 * TeleOp and Autonomous share this one drivetrain: the same motors, odometry (Pinpoint) and pose.
 * Motor names, directions and tuning live in {@link Constants}. Autonomous passes
 * {@link #getFollower()} to the SolversLib Pedro commands (FollowPathCommand, TurnCommand, ...).
 * <p>
 * {@link #periodic()} calls {@code follower.update()} every loop, so OpModes must NOT also call it.
 */
public class Drivetrain extends SubsystemBase {
    private final Follower follower;
    private final Telemetry telemetry;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        follower = Constants.createFollower(hardwareMap);
    }

    /**
     * Field-centric drive. Inputs are in Pedro's frame: +forward, +left, +counterclockwise
     * (see DriveWithController for the gamepad conversion).
     */
    public void driveFieldCentric(double forward, double left, double turn) {
        follower.manual(ManualDrive.fieldCentric(forward, left, turn, follower.pose().heading()));
    }

    /** Robot-centric drive, same input frame as {@link #driveFieldCentric}. */
    public void driveRobotCentric(double forward, double left, double turn) {
        follower.manual(forward, left, turn);
    }

    public void stop() {
        follower.manual(0, 0, 0);
    }

    /** Makes the robot's current facing the new "forward" for field-centric driving. */
    public void resetHeading() {
        Pose pose = follower.pose();
        follower.setPose(new Pose(pose.x(), pose.y(), 0));
    }

    public Pose getPose() {
        return follower.pose();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public Follower getFollower() {
        return follower;
    }

    @Override
    public void periodic() {
        follower.update();

        Pose pose = follower.pose();
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f°",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }
}
