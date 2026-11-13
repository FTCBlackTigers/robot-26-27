package org.firstinspires.ftc.teamcode.opmodes;
package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoTest extends LinearOpMode {

    private Robot robot;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.6072, 2.7856, 90);
    private final Pose path1 = poseFactory.of(54.944, 28.0802, 180);
    private final Pose point2 = poseFactory.of(103.5837, 31.4033, -176.0916);
    private final Pose point3 = poseFactory.of(62.8692, 2.7257, -144.8408);

    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3())
        );
    }

    @Override
    public void runOpMode() {

        // Reset Pedro Ivy scheduler
        Scheduler.reset();

        // Create the robot and all its subsystems
        robot = new Robot(hardwareMap, telemetry);

        // Reuse the drivetrain's Follower
        follower = robot.drivetrain.getFollower();

        // Set autonomous starting position
        robot.drivetrain.setPose(start);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Schedule autonomous routine
        schedule(autoRoutine());

        while (opModeIsActive()) {

            // Run SolversLib scheduler:
            // Calls Drivetrain.periodic(), which calls follower.update()
            CommandScheduler.getInstance().run();

            // Run Pedro Ivy autonomous commands
            Scheduler.execute();

            // Telemetry
            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData(
                    "Heading",
                    Math.toDegrees(follower.pose().heading())
            );

            telemetry.addData("Busy", follower.isBusy());

            if (follower.currentPath() != null) {
                telemetry.addData(
                        "Distance remaining",
                        follower.distanceToEndpoint()
                );
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }

        robot.drivetrain.stop();
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).reverseTangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).tangent();
    }
}
