package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.adapters.IvyCommandAdapter;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeIn;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class Aviv_mezalem_manolim extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Shooter shooterSubsystem = new Shooter(hardwareMap, telemetry);
    private final ShooterCommand shooterCommand = new ShooterCommand(shooterSubsystem);
    private final Intake intakeSubsystem = new Intake(hardwareMap, telemetry);
    private final IntakeIn intakeInCommand = new IntakeIn(intakeSubsystem);


    private final Pose start = poseFactory.of(59.5553, 8.9481, 90);
    private final Pose path1 = poseFactory.of(59.2546, 28.0802, 90);
    private final Pose point2Start = poseFactory.of(59.2546, 28.0802, 180);
    private final Pose point2 = poseFactory.of(20.4854, 9.5515, 180);
    private final Pose point3Start = poseFactory.of(20.4854, 9.5515, 180);
    private final Pose point3 = poseFactory.of(8.7688, 9.9548, 90);
    private final Pose point4Start = poseFactory.of(8.7688, 9.9548, 90);
    private final Pose point4 = poseFactory.of(33.7537, 109.3139, 90);
    private final Pose point5Start = poseFactory.of(33.7537, 109.3139, 270);
    private final Pose point5 = poseFactory.of(58.4396, 111.6407, 270);
    private final Pose point6Start = poseFactory.of(58.4396, 111.6407, 270);
    private final Pose point6 = poseFactory.of(47.8612, 135.2046, 90);
    private final Pose point7Start = poseFactory.of(47.8612, 135.2046, 90);
    private final Pose point7 = poseFactory.of(58.247, 110.6926, 270);
    private final Pose point8Start = poseFactory.of(58.247, 110.6926, 270);
    private final Pose point8 = poseFactory.of(15.0118, 109.2338, 270);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                IvyCommandAdapter.fromSolversLib(shooterCommand),
                follow(follower, path2()),
                IvyCommandAdapter.fromSolversLib(intakeInCommand),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                IvyCommandAdapter.fromSolversLib(intakeInCommand),
                follow(follower, path7()),
                follow(follower, path8())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point8Start, point8).linear(point8Start, point8);
    }
}
