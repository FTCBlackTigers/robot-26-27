package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Intake. PLACEHOLDER: hardware is commented out until the mechanism is built.
 * Uncomment the lines, add the imports, and match the name to the Driver Station configuration.
 */
public class Intake extends SubsystemBase {
    private final Telemetry telemetry;

    // private final DcMotorEx motor;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // motor = hardwareMap.get(DcMotorEx.class, "INTAKE");
        // motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intake() {
        // motor.setPower(1);
    }

    public void outtake() {
        // motor.setPower(-1);
    }

    public void stop() {
        // motor.setPower(0);
    }

    @Override
    public void periodic() {
        // telemetry.addData("Intake power", motor.getPower());
    }
}
