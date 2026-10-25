package org.firstinspires.ftc.teamcode.smokeTests.commandTests;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.drive.DriveWithController;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Smallest useful robot: only the drivetrain. Checks that the command system, the Pedro follower
 * and the driver controls work before the other subsystems are added back.
 */
public class RobotDriveTest {
    public final Drivetrain drivetrain;

    public RobotDriveTest(HardwareMap hardwareMap, Telemetry telemetry) {
        // Clear anything a previous OpMode left in the scheduler
        CommandScheduler.getInstance().reset();
        drivetrain = new Drivetrain(hardwareMap, telemetry);
        CommandScheduler.getInstance().registerSubsystem(drivetrain);
    }

    /** gamepad1 only: sticks = drive, OPTIONS = reset heading. */
    public void configureTeleOp(Gamepad gamepad1) {
        GamepadEx driver = new GamepadEx(gamepad1);

        drivetrain.setDefaultCommand(new DriveWithController(drivetrain, driver));
        driver.getGamepadButton(GamepadKeys.Button.OPTIONS)
                .whenPressed(new InstantCommand(drivetrain::resetHeading));
    }
}
