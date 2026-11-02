package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Main driver-controlled OpMode. Everything (subsystems, controls) lives in {@link Robot};
 * see {@link Robot#configureTeleOp} for the button map.
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends CommandOpMode {
    @Override
    public void initialize() {
        Robot robot = new Robot(hardwareMap, telemetry);
        robot.configureTeleOp(gamepad1, gamepad2);
    }

    @Override
    public void run() {
        // Runs the command scheduler, which calls every subsystem's periodic() (that's where
        // telemetry lines are added and the Pedro follower is updated)
        super.run();
        telemetry.update();
    }
}
