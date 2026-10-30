package org.firstinspires.ftc.teamcode.smokeTests.commandTests;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeIn;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Smallest useful robot: only the drivetrain. Checks that the command system, the Pedro follower
 * and the driver controls work before the other subsystems are added back.
 */
public class IntakeInTest {
    public final Intake intake;

    public IntakeInTest(HardwareMap hardwareMap, Telemetry telemetry) {
        // Clear anything a previous OpMode left in the scheduler
        CommandScheduler.getInstance().reset();
        intake = new Intake(hardwareMap, telemetry);
        CommandScheduler.getInstance().registerSubsystem(drivetrain);
    }

    /** gamepad1 only: sticks = drive, OPTIONS = reset heading. */
    public void configureTeleOp(Gamepad gamepad1) {
        GamepadEx gamepad = new GamepadEx(gamepad1);

        drivetrain.setDefaultCommand(new IntakeIn(intake, gamepad));
        driver.getGamepadButton(GamepadKeys.Button.OPTIONS)
                .whenPressed(new InstantCommand(drivetrain::resetHeading));
    }
}
