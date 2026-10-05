package org.firstinspires.ftc.teamcode.opmodes.utilities;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

/**
 * Checks the Limelight 3A: shows connection, pipeline, target angles, AprilTag IDs and botpose.
 * gamepad1 dpad up/down = next/previous pipeline (0-9).
 * Needs the Limelight in the Driver Station configuration, named "limelight".
 * Hidden for now (vision isn't used yet): remove @Disabled to show it on the Driver Station.
 */
@Disabled
@TeleOp(name = "Limelight Test", group = "Utilities")
public class LimelightTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Limelight limelight = new Limelight(hardwareMap, telemetry);
        int pipeline = Limelight.DEFAULT_PIPELINE;
        boolean lastUp = false, lastDown = false;

        telemetry.setMsTransmissionInterval(11);

        while (opModeInInit()) {
            limelight.periodic();
            telemetry.addLine(limelight.isConnected()
                    ? "Press START"
                    : "Not found: add it to the robot configuration as \"" + Limelight.DEVICE_NAME + "\"");
            telemetry.update();
        }

        while (opModeIsActive()) {
            // Not a command OpMode, so periodic() is called by hand (it also adds a summary line)
            limelight.periodic();

            if (gamepad1.dpad_up && !lastUp && pipeline < 9) limelight.switchPipeline(++pipeline);
            if (gamepad1.dpad_down && !lastDown && pipeline > 0) limelight.switchPipeline(--pipeline);
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            telemetry.addData("Pipeline (requested / running)", "%d / %d", pipeline, limelight.getPipelineIndex());
            telemetry.addData("tx / ty / ta", "%.2f / %.2f / %.2f", limelight.getTx(), limelight.getTy(), limelight.getTa());

            for (LLResultTypes.FiducialResult tag : limelight.getAprilTags()) {
                telemetry.addData("AprilTag " + tag.getFiducialId(), "x %.1f°, y %.1f°",
                        tag.getTargetXDegrees(), tag.getTargetYDegrees());
            }

            Pose3D botpose = limelight.getBotpose();
            telemetry.addData("Botpose", botpose != null ? botpose.toString() : "none");
            telemetry.update();
        }

        limelight.stop();
    }
}
