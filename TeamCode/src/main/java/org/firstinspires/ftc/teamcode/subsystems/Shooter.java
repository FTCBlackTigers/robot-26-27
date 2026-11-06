package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Shooter (flywheel), one speed. Motor "SHOOTER", its encoder cable in the encoder port with the same number.
 * <p>
 * setVelocity (encoder ticks per second) instead of setPower keeps shots the same as the battery drains.
 */
public class Shooter extends SubsystemBase {
    // Ticks per second. Must be a speed the robot can hold, or it just runs at full power with nothing
    // left to recover after a shot. Tops out near 1580 (2026-11-06, Shooter Test), drag not yet found.
    public static double TARGET_VELOCITY = 1500;

    private final Telemetry telemetry;

    private final DcMotorEx flywheel;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        flywheel = hardwareMap.get(DcMotorEx.class, "SHOOTER");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void spinUp() {
        flywheel.setVelocity(TARGET_VELOCITY);
    }

    public void stop() {
        flywheel.setVelocity(0);
    }

    @Override
    public void periodic() {
        telemetry.addData("Shooter velocity", flywheel.getVelocity());
    }
}
