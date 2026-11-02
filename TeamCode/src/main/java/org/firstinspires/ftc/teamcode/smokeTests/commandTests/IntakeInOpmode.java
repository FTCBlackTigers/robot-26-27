package org.firstinspires.ftc.teamcode.smokeTests.commandTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

@TeleOp(name = "Smoke: IntakeIn Command Test", group = "Smoke Tests")
public class IntakeInOpmode extends CommandOpMode {
    @Override
    public void initialize() {
        IntakeInTest robot = new IntakeInTest(hardwareMap, telemetry);
        robot.configureTeleOp(gamepad1);
    }

    @Override
    public void run() {
        // Poll the bumper bindings and publish intake power telemetry.
        super.run();
        telemetry.update();
    }
}
