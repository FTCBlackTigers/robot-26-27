package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.drive.DriveWithController;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeIn;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * The whole robot: creates every subsystem once and holds the TeleOp button bindings.
 * Every OpMode (TeleOp and Autonomous) builds one of these.
 */
public class Robot {
    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Shooter shooter;
    // Vision is off for now. To turn it on: uncomment the Limelight lines here and the AimAtTarget
    // binding below, add `limelight` to registerSubsystem, and remove @Disabled from LimelightTest.
    // public final Limelight limelight;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        // Clear anything a previous OpMode left in the scheduler
        CommandScheduler.getInstance().reset();

        drivetrain = new Drivetrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        // limelight = new Limelight(hardwareMap, telemetry);

        // Registered subsystems get periodic() called every loop
        CommandScheduler.getInstance().registerSubsystem(drivetrain, intake, shooter);
    }

    /**
     * gamepad1 (driver): sticks = drive, OPTIONS = reset heading.
     * gamepad2 (operator): RB hold = intake, LB hold = outtake, A = shooter on, B = shooter off.
     */
    public void configureTeleOp(Gamepad gamepad1, Gamepad gamepad2) {
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);

        // Driver
        drivetrain.setDefaultCommand(new DriveWithController(drivetrain, driver));
        driver.getGamepadButton(GamepadKeys.Button.OPTIONS)
                .whenPressed(new InstantCommand(drivetrain::resetHeading));
        // driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
        //         .whenHeld(new AimAtTarget(drivetrain, limelight));

        // Operator
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new IntakeIn(intake));
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new StartEndCommand(intake::outtake, intake::stop, intake));
        operator.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(shooter::spinUp, shooter));
        operator.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(shooter::stop, shooter));
    }
}
