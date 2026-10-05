package org.firstinspires.ftc.teamcode.opmodes.utilities;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * Checks the Limelight 3A: shows connection, pipeline, target angles, AprilTag IDs and botpose.
 * gamepad1 dpad up/down = next/previous pipeline (0-9).
 * Needs the Limelight in the Driver Station configuration, named "limelight".
 */
@TeleOp(name = "Limelight Test", group = "Utilities")
public class LimelightTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        VisionSubsystem vision = new VisionSubsystem(hardwareMap);
        int pipeline = VisionSubsystem.DEFAULT_PIPELINE;
        boolean lastUp = false, lastDown = false;

        telemetry.setMsTransmissionInterval(11);

        while (opModeInInit()) {
            vision.periodic();
            telemetry.addData("Limelight connected", vision.isConnected());
            telemetry.addLine(vision.isConnected()
                    ? "Press START"
                    : "Not found: add it to the robot configuration as \"" + VisionSubsystem.DEVICE_NAME + "\"");
            telemetry.update();
        }

        while (opModeIsActive()) {
            // Not a command OpMode, so periodic() is called by hand
            vision.periodic();

            if (gamepad1.dpad_up && !lastUp && pipeline < 9) vision.switchPipeline(++pipeline);
            if (gamepad1.dpad_down && !lastDown && pipeline > 0) vision.switchPipeline(--pipeline);
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            telemetry.addData("Connected", vision.isConnected());
            telemetry.addData("Pipeline (requested / running)", "%d / %d", pipeline, vision.getPipelineIndex());
            telemetry.addData("Has target", vision.hasTarget());
            telemetry.addData("tx / ty / ta", "%.2f / %.2f / %.2f", vision.getTx(), vision.getTy(), vision.getTa());

            for (LLResultTypes.FiducialResult tag : vision.getAprilTags()) {
                telemetry.addData("AprilTag " + tag.getFiducialId(), "x %.1f°, y %.1f°",
                        tag.getTargetXDegrees(), tag.getTargetYDegrees());
            }

            Pose3D botpose = vision.getBotpose();
            telemetry.addData("Botpose", botpose != null ? botpose.toString() : "none");
            telemetry.update();
        }

        vision.stop();
    }
}
