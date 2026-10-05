package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * Main driver-controlled OpMode.
 * <p>
 * gamepad1 (driver): sticks = field-centric drive, OPTIONS = reset heading.
 * <br>
 * gamepad2 (operator): RB hold = intake, LB hold = outtake, A = shooter spin up, B = shooter stop,
 * X = feed. Intake and shooter are placeholders, so those buttons only change the state shown on
 * telemetry until the hardware code is uncommented.
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends CommandOpMode {
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;

    private final TelemetryData telemetryData = new TelemetryData(telemetry);

    @Override
    public void initialize() {
        super.reset();

        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        register(drive, intake, shooter);

        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);

        // Driver
        // Pedro uses +x forwards, +y left and counterclockwise positive, while the sticks read positive
        // to the right and down, so all three axes are negated (same as PedroTeleOpSample)
        drive.setDefaultCommand(new RunCommand(() -> drive.driveFieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        ), drive));

        driver.getGamepadButton(GamepadKeys.Button.OPTIONS)
                .whenPressed(new InstantCommand(drive::resetHeading));

        // Operator: intake
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(intake::intake, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(intake::outtake, intake))
                .whenReleased(new InstantCommand(intake::stop, intake));

        // Operator: shooter
        operator.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(shooter::spinUp, shooter));
        operator.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(shooter::stop, shooter));
        operator.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(shooter::feed, shooter))
                .whenReleased(new InstantCommand(shooter::retractFeeder, shooter));
    }

    @Override
    public void run() {
        // Runs the command scheduler, which also calls every subsystem's periodic()
        // (DriveSubsystem.periodic() updates the Pedro follower)
        super.run();

        telemetryData.addData("X", drive.getPose().x());
        telemetryData.addData("Y", drive.getPose().y());
        telemetryData.addData("Heading", drive.getPose().heading());
        telemetryData.addData("Intake", intake.getState());
        telemetryData.addData("Shooter", shooter.getState());
        telemetryData.update();
    }
}
