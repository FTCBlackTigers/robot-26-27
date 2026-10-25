package org.firstinspires.ftc.teamcode.smokeTests.commandTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

@TeleOp(name = "Smoke: Robot Drive", group = "Smoke Tests")
public class RobotDriveOpmode extends CommandOpMode {
    @Override
    public void initialize() {
        RobotDriveTest robot = new RobotDriveTest(hardwareMap, telemetry);
        robot.configureTeleOp(gamepad1);
    }

    @Override
    public void run() {
        // Runs the command scheduler, which calls every subsystem's periodic() (that's where
        // telemetry lines are added and the Pedro follower is updated)
        super.run();
        telemetry.update();
    }
}
