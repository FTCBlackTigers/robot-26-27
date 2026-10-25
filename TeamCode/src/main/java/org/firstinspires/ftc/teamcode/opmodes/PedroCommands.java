package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeIn;

/**
 * Pedro Pathing 3 paste-and-run auto.
 * 1. Paste the PoseFactory / Pose / Path code (e.g. from the path visualizer) into the PASTE ZONE.
 * 2. Write the auto in {@link #routine}: drive(path) and robot commands, top to bottom, in order.
 * 3. Point {@link #startPose()} at the first path.
 */
@Autonomous(name = "test", group = "tets")
public class PedroCommands extends AutonomousBase {

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

    /** Where the robot starts: the start of your first path. */
    @Override
    protected Pose startPose() {
        return firstPath().get(0);
    }

    /** The auto, top to bottom. Each line waits for the one before it to finish. */
    @Override
    protected Command routine(Robot robot) {
        return new SequentialCommandGroup(
                drive(firstPath()),

                // Intake for 1 second while standing still (IntakeIn never stops by itself, so give it a timeout)
                new IntakeIn(robot.intake).withTimeout(1000),

                // Intake WHILE driving: the intake stops when the path ends
                drive(secondPath()).deadlineWith(new IntakeIn(robot.intake)),

                // One-shot actions
                new InstantCommand(robot.shooter::spinUp, robot.shooter),
                new WaitCommand(500)
        );
    }

    /** Follow a path (Drivetrain updates the follower every loop). */
    private Command drive(Path path) {
        return new FollowPathCommand(robot.drivetrain.getFollower(), path);
    }
}
