package org.firstinspires.ftc.teamcode.opmodes.utilities;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

/**
 * Checks the Limelight 3A: shows connection, pipeline, target angles, AprilTag IDs and the robot's
 * position from the tags. gamepad1 dpad up/down = next/previous pipeline (0-9).
 * <p>
 * Checking the position (once, on a real field, with the season's field map in the Limelight):
 * <ol>
 *     <li>Point the robot at Pedro's 0° (toward +x) where it can see tags. If the
 *     "FIELD_ROTATION_DEG should be" line doesn't match Limelight.FIELD_ROTATION_DEG, change it to that.</li>
 *     <li>Put the robot on a spot you measured (inches from Pedro's (0, 0) corner): "Robot (Pedro)"
 *     should match within an inch or two. If it's off by the same amount everywhere, the camera's
 *     position on the robot is wrong in the Limelight web UI.</li>
 * </ol>
 * Needs the Limelight in the Driver Station configuration, named "limelight".
 */
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
            telemetry.addData("Result age", "%d ms (over %d = ignored)", limelight.getResultAgeMs(), Limelight.MAX_RESULT_AGE_MS);
            telemetry.addData("tx / ty / ta", "%.2f / %.2f / %.2f", limelight.getTx(), limelight.getTy(), limelight.getTa());
            telemetry.addData("Aim tx (middle of all tags)", "%.2f", limelight.getAimTx());

            for (LLResultTypes.FiducialResult tag : limelight.getAprilTags()) {
                telemetry.addData("AprilTag " + tag.getFiducialId(), "x %.1f°, y %.1f°",
                        tag.getTargetXDegrees(), tag.getTargetYDegrees());
            }

            Pose pose = limelight.getPedroPose();
            telemetry.addData("Robot (Pedro)", pose != null
                    ? String.format("x %.1f  y %.1f  heading %.0f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()))
                    : "none (no field map tag in view)");
            Pose3D botpose = limelight.getBotpose();
            if (pose != null) {
                // Pedro heading = Limelight yaw + rotation, so facing Pedro 0° means rotation = -yaw
                double yaw = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
                telemetry.addData("FIELD_ROTATION_DEG should be (facing Pedro 0°)", "%d (set: %.0f)",
                        Math.floorMod(Math.round(-yaw / 90) * 90, 360L), Limelight.FIELD_ROTATION_DEG);
            }
            telemetry.addData("Botpose (Limelight, meters)", botpose != null ? botpose.toString() : "none");
            telemetry.update();
        }

        limelight.stop();
    }
}
