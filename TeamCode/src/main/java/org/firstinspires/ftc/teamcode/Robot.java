package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.automations.Shoot;
import org.firstinspires.ftc.teamcode.commands.drive.AimAtTarget;
import org.firstinspires.ftc.teamcode.commands.drive.DriveWithController;
import org.firstinspires.ftc.teamcode.commands.drive.ResetHeading;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeIn;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOut;
import org.firstinspires.ftc.teamcode.commands.shooter.SpinUp;
import org.firstinspires.ftc.teamcode.commands.shooter.StopShooter;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * The whole robot: creates every subsystem once and holds the TeleOp button bindings, so OpModes
 * stay a few lines long. Every OpMode (TeleOp and Autonomous) builds one of these.
 * <p>
 * Each subsystem adds its own telemetry lines in periodic(); the OpMode calls telemetry.update()
 * once per loop after the scheduler runs.
 */
public class Robot {
    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Shooter shooter;
    public final Limelight limelight;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        // The scheduler is a singleton that outlives OpModes: clear anything a previous OpMode left
        // behind (registered subsystems, button bindings, running commands)
        CommandScheduler.getInstance().reset();

        drivetrain = new Drivetrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        limelight = new Limelight(hardwareMap, telemetry);

        // Registered subsystems get periodic() called every loop
        CommandScheduler.getInstance().registerSubsystem(drivetrain, intake, shooter, limelight);
    }

    /**
     * TeleOp controls.
     * <p>
     * gamepad1 (driver): sticks = field-centric drive, OPTIONS = reset heading,
     * RB hold = aim at Limelight target (shell, does nothing yet).
     * <br>
     * gamepad2 (operator): RB hold = intake, LB hold = outtake, A = spin up shooter, B = stop shooter,
     * X = shoot one piece.
     */
    public void configureTeleOp(Gamepad gamepad1, Gamepad gamepad2) {
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);

        // ------------ Driver ------------
        drivetrain.setDefaultCommand(new DriveWithController(drivetrain, driver));

        driver.getGamepadButton(GamepadKeys.Button.OPTIONS)
                .whenPressed(new ResetHeading(drivetrain));
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new AimAtTarget(drivetrain, limelight));

        // ------------ Operator: intake ------------
        // whenHeld = start on press, cancel on release
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new IntakeIn(intake));
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new IntakeOut(intake));

        // ------------ Operator: shooter ------------
        operator.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SpinUp(shooter));
        operator.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new StopShooter(shooter));
        operator.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new Shoot(shooter));
    }
}
