package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Base for every Autonomous. A real auto only says where it starts and what it does:
 * <pre>
 * &#64;Autonomous(name = "Red Goal", group = "Competition")
 * public class RedGoal extends AutonomousBase {
 *     protected Pose startPose() { return new Pose(122, 124, Math.toRadians(37)); }
 *
 *     protected Command routine(Robot robot) {
 *         Path toScore = line(startPose(), scorePose).linear(startPose(), scorePose);
 *         return new SequentialCommandGroup(
 *                 new FollowPathCommand(robot.drivetrain.getFollower(), toScore),
 *                 new Shoot(robot.shooter)
 *         );
 *     }
 * }
 * </pre>
 * Paths: see samples/PedroAutoSample. The routine is built during INIT and starts on START
 * (the scheduler doesn't run during INIT).
 */
public abstract class AutonomousBase extends CommandOpMode {
    protected Robot robot;

    /** Where the robot is placed on the field, in Pedro coordinates (inches, radians). */
    protected abstract Pose startPose();

    /** The whole autonomous as one command, usually a SequentialCommandGroup. */
    protected abstract Command routine(Robot robot);

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, telemetry);
        robot.drivetrain.setPose(startPose());
        schedule(routine(robot));
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}
