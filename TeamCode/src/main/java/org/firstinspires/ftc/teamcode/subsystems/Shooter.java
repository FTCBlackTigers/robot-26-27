package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Shooter (flywheel). PLACEHOLDER: hardware is commented out until the mechanism is built.
 * Uncomment the lines, add the imports, and match the name to the Driver Station configuration.
 * <p>
 * setVelocity (encoder ticks per second) instead of setPower keeps shots the same as the battery drains.
 */
public class Shooter extends SubsystemBase {
    public static double TARGET_VELOCITY = 1500; // ticks per second, tune on the robot

    private final Telemetry telemetry;

    // private final DcMotorEx flywheel;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // flywheel = hardwareMap.get(DcMotorEx.class, "SHOOTER");
        // flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void spinUp() {
        // flywheel.setVelocity(TARGET_VELOCITY);
    }

    public void stop() {
        // flywheel.setVelocity(0);
    }

    @Override
    public void periodic() {
        // telemetry.addData("Shooter velocity", flywheel.getVelocity());
    }
}
