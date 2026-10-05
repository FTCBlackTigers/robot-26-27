package org.firstinspires.ftc.teamcode.opmodes.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Quick hardware test, no controller: all 6 motors forward for 5 s, stop 1 s, backward for 5 s.
 * Driver Station names: RF, RB, LF, LB (drive), INTAKE, SHOOTER.
 * Listed under TeleOp (not Autonomous) so it has no 30 s auto timer, but it still needs no gamepad.
 */
@TeleOp(name = "Motor Test", group = "Utilities")
public class MotorTest extends LinearOpMode {
    private static final double POWER = 0.4;
    private static final long RUN_MS = 5000;

    private DcMotor[] motors;

    @Override
    public void runOpMode() {
        DcMotor rf = hardwareMap.get(DcMotor.class, "RF");
        DcMotor rb = hardwareMap.get(DcMotor.class, "RB");
        DcMotor lf = hardwareMap.get(DcMotor.class, "LF");
        DcMotor lb = hardwareMap.get(DcMotor.class, "LB");
        DcMotor intake = hardwareMap.get(DcMotor.class, "INTAKE");
        DcMotor shooter = hardwareMap.get(DcMotor.class, "SHOOTER");

        // Left side is mirrored, so reverse it to make "forward" drive forward.
        // If the robot spins instead of driving straight, swap these to the right side.
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new DcMotor[]{rf, rb, lf, lb, intake, shooter};
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Ready. Press START: forward 5s, stop 1s, backward 5s");
        telemetry.update();
        waitForStart();

        runFor("Forward", POWER, RUN_MS);
        runFor("Stopped", 0, 1000);
        runFor("Backward", -POWER, RUN_MS);
        setAll(0);
    }

    private void runFor(String label, double power, long ms) {
        setAll(power);
        long end = System.currentTimeMillis() + ms;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            telemetry.addData("Phase", label);
            telemetry.addData("Power", power);
            telemetry.update();
            idle();
        }
    }

    private void setAll(double power) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }
}
