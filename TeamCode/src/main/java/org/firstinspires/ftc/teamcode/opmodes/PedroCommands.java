package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Pedro Pathing 3 paste-and-run auto. Paste the PoseFactory / Pose / Path code (e.g. from the path
 * visualizer) into the PASTE ZONE, list the paths in {@link #route()}, done.
 * The robot's start pose is the start of the first path in {@link #route()}.
 */
@Autonomous(name = "test", group = "tets")
public class PedroCommands extends CommandOpMode {
    private Follower follower;
    private final TelemetryData telemetryData = new TelemetryData(telemetry);

    // ===================== PASTE ZONE: replace everything down to END PASTE ZONE =====================

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(56, 8, 90);
    private final Pose firstPose = poseFactory.of(56, 36, 180);
    private final Pose secondPose = poseFactory.of(108.7565, 36.2015, 90);

    public Path firstPath() {
        return Paths.line(startPose, firstPose).linear(startPose, firstPose);
    }

    public Path secondPath() {
        return Paths.line(firstPose, secondPose).constant(secondPose);
    }

    // ===================== END PASTE ZONE =====================

    /** The paths to drive, in order. Put the path methods from the paste zone here. */
    private Path[] route() {
        return new Path[] {
                firstPath(),
                secondPath(),
        };
    }

    @Override
    public void initialize() {
        super.reset();

        follower = Constants.createFollower(hardwareMap);

        Path[] route = route();
        follower.setPose(route[0].get(0)); // start of the first path

        SequentialCommandGroup auto = new SequentialCommandGroup();
        for (Path path : route) {
            auto.addCommands(new FollowPathCommand(follower, path));
        }
        schedule(auto);
    }

    @Override
    public void run() {
        super.run();

        // The follower needs to be updated every loop
        follower.update();

        telemetryData.addData("X", follower.pose().x());
        telemetryData.addData("Y", follower.pose().y());
        telemetryData.addData("Heading", follower.pose().heading());
        telemetryData.update();
    }
}
