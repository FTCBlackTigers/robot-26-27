package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Collections;
import java.util.List;

/**
 * Limelight 3A camera, using the driver built into the FTC SDK (no extra library needed).
 * <p>
 * Setup: plug the Limelight into the Control Hub's blue USB 3.0 port, then in the Driver Station go to
 * Configure Robot, press Scan, and name the new Ethernet Device {@value #DEVICE_NAME}. Pipelines
 * (AprilTags, color, neural nets) are built in the Limelight web UI: plug the Limelight into a laptop
 * by USB and open http://limelight.local:5801 (docs.limelightvision.io, Limelight 3A Quick-Start).
 * Use the "Limelight Test" OpMode to check it on the robot.
 * <p>
 * If the Limelight isn't in the configuration, this subsystem stays inactive instead of crashing
 * the OpMode: {@link #isConnected()} is false and every getter reports "no target".
 */
public class Limelight extends SubsystemBase {
    public static final String DEVICE_NAME = "limelight";
    public static int DEFAULT_PIPELINE = 0;
    /**
     * Results older than this (ms) don't count. The SDK keeps handing back the last result forever, so
     * without this a frozen or unplugged camera would look like a target that never moves, and
     * AimAtTarget would keep turning. A healthy Limelight is ~10-30 ms old.
     */
    public static long MAX_RESULT_AGE_MS = 100;

    private final Limelight3A limelight; // null when not configured
    private final Telemetry telemetry;
    private LLResult latest;
    private boolean hasTarget; // decided once per loop in periodic(), so every getter agrees

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.tryGet(Limelight3A.class, DEVICE_NAME);
        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(DEFAULT_PIPELINE);
            // getLatestResult() returns nothing useful until start() is called
            limelight.start();
        }
    }

    /** True when the Limelight is configured and answering. */
    public boolean isConnected() {
        return limelight != null && limelight.isConnected();
    }

    /** True when the current pipeline sees a target and the result is fresh. */
    public boolean hasTarget() {
        return hasTarget;
    }

    /** How old the latest result is in ms (-1 before the first one). */
    public long getResultAgeMs() {
        return latest != null ? latest.getStaleness() : -1;
    }

    /** Horizontal angle to the target in degrees (0 when there is no target). */
    public double getTx() {
        return hasTarget() ? latest.getTx() : 0;
    }

    /** Vertical angle to the target in degrees (0 when there is no target). */
    public double getTy() {
        return hasTarget() ? latest.getTy() : 0;
    }

    /** Target area, % of the image (0 when there is no target). */
    public double getTa() {
        return hasTarget() ? latest.getTa() : 0;
    }

    /** AprilTags seen by an AprilTag pipeline. Each has getFiducialId(), getTargetXDegrees(), ... */
    public List<LLResultTypes.FiducialResult> getAprilTags() {
        return hasTarget() ? latest.getFiducialResults() : Collections.<LLResultTypes.FiducialResult>emptyList();
    }

    /**
     * Horizontal angle in degrees to the middle of all visible AprilTags (0 when there is no target).
     * The hive faces have two tags side by side, so this aims between them instead of at whichever
     * one looks bigger. Which tags count is set by the pipeline's ID Filters in the Limelight web UI.
     * Falls back to {@link #getTx()} for non-AprilTag pipelines.
     */
    public double getAimTx() {
        List<LLResultTypes.FiducialResult> tags = getAprilTags();
        if (tags.isEmpty()) return getTx();

        double sum = 0;
        for (LLResultTypes.FiducialResult tag : tags) {
            sum += tag.getTargetXDegrees();
        }
        return sum / tags.size();
    }

    /** ID of the first AprilTag seen, or -1 when none. */
    public int getAprilTagId() {
        List<LLResultTypes.FiducialResult> tags = getAprilTags();
        return tags.isEmpty() ? -1 : tags.get(0).getFiducialId();
    }

    /**
     * Robot pose on the field from AprilTags (MegaTag1), or null with no target. Position is in
     * meters: use {@code getPosition().toUnit(DistanceUnit.INCH)} for Pedro's inches.
     * Needs the field map + camera position set in the Limelight web UI. For MegaTag2 call
     * {@link #updateRobotYaw} every loop and use {@code getBotpose_MT2()} instead.
     */
    public Pose3D getBotpose() {
        return hasTarget() ? latest.getBotpose() : null;
    }

    /** Feeds the robot heading (degrees) to the Limelight for MegaTag2 localization. */
    public void updateRobotYaw(double yawDegrees) {
        if (limelight != null) limelight.updateRobotOrientation(yawDegrees);
    }

    public void switchPipeline(int index) {
        if (limelight != null) limelight.pipelineSwitch(index);
    }

    public int getPipelineIndex() {
        return latest != null ? latest.getPipelineIndex() : -1;
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }

    @Override
    public void periodic() {
        if (limelight != null) latest = limelight.getLatestResult();
        hasTarget = latest != null && latest.isValid() && latest.getStaleness() <= MAX_RESULT_AGE_MS;

        if (!isConnected()) {
            telemetry.addData("Limelight", "not connected");
        } else if (!hasTarget()) {
            telemetry.addData("Limelight", "no target");
        } else {
            telemetry.addData("Limelight", "tx %.1f°  ty %.1f°  tag %d", getTx(), getTy(), getAprilTagId());
        }
    }
}
