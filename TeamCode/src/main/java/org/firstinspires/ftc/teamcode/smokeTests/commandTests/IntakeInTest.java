package org.firstinspires.ftc.teamcode.smokeTests.commandTests;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeIn;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOut;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Intake-only smoke test. Requires only the INTAKE motor, without drive hardware or odometry.
 */
public class IntakeInTest {
    public final Intake intake;

    public IntakeInTest(HardwareMap hardwareMap, Telemetry telemetry) {
        // Clear anything a previous OpMode left in the scheduler
        CommandScheduler.getInstance().reset();
        intake = new Intake(hardwareMap, telemetry);
        CommandScheduler.getInstance().registerSubsystem(intake);
    }

    /** gamepad1: hold RB to intake, hold LB to outtake; release to stop. */
    public void configureTeleOp(Gamepad gamepad1) {
        GamepadEx gamepad = new GamepadEx(gamepad1);

        // No default command: the intake must stay stopped when neither bumper is held.
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new IntakeIn(intake));
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new IntakeOut(intake));
    }
}
