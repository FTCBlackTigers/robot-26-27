package org.firstinspires.ftc.teamcode.opmodes.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Finds out why the shooter tops out, and measures its gear ratio. Only needs SHOOTER, so the drive
 * motors stay off. The motor only runs while a button is held.
 * <p>
 * gamepad1: hold A = true full power (no speed control), hold B = speed control at
 * {@link #VELOCITY_TARGET}, Y = zero the ticks and top speed (while not running).
 * <p>
 * Top speed: hold A for about 5 s, then the same with B. A stock 5203 should pass 2500 ticks/s.
 * <ul>
 *     <li>A stays near B's speed with high current: mechanical drag.</li>
 *     <li>A is well above B: the speed control isn't using full power, a software fix.</li>
 *     <li>A stays low with low current: suspect the motor or the encoder reading.</li>
 * </ul>
 * Gear ratio: press Y, turn the flywheel exactly 5 turns by hand, then ticks / 5 / 28 = total ratio.
 */
@TeleOp(name = "Shooter Test", group = "Utilities")
public class ShooterTest extends LinearOpMode {
    private static final double VELOCITY_TARGET = 2100; // ticks per second, above what it reaches now

    private DcMotorEx flywheel;
    private double topSpeed = 0;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "SHOOTER");
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        resetMeasurements();

        telemetry.addLine("Ready. Keep clear of the flywheel, then press START");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            String state;
            if (gamepad1.a) {
                coastMode();
                flywheel.setPower(1);
                state = "FULL POWER";
            } else if (gamepad1.b) {
                flywheel.setVelocity(VELOCITY_TARGET); // switches to speed control by itself
                state = "SPEED CONTROL " + VELOCITY_TARGET;
            } else {
                coastMode();
                flywheel.setPower(0);
                if (gamepad1.y) {
                    resetMeasurements();
                }
                state = "Stopped";
            }

            double velocity = flywheel.getVelocity();
            topSpeed = Math.max(topSpeed, Math.abs(velocity));
            PIDFCoefficients pidf = flywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addLine("Hold A = full power, hold B = speed control, Y = zero");
            telemetry.addData("State", state);
            telemetry.addData("Velocity (ticks/s)", "%.0f", velocity);
            telemetry.addData("Top speed (ticks/s)", "%.0f", topSpeed);
            telemetry.addData("Current (A)", "%.2f", flywheel.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Battery (V)", "%.2f", battery.getVoltage());
            telemetry.addData("Ticks", flywheel.getCurrentPosition());
            telemetry.addData("PIDF", "P %.2f  I %.2f  D %.2f  F %.2f", pidf.p, pidf.i, pidf.d, pidf.f);
            telemetry.update();
        }
        flywheel.setPower(0);
    }

    /** Raw power mode. Power 0 here coasts, while speed control would actively brake to 0. */
    private void coastMode() {
        if (flywheel.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void resetMeasurements() {
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topSpeed = 0;
    }
}
