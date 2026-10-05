package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

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
public class VisionSubsystem extends SubsystemBase {
    public static final String DEVICE_NAME = "limelight";
    public static int DEFAULT_PIPELINE = 0;

    private final Limelight3A limelight; // null when not configured
    private LLResult latest;

    public VisionSubsystem(HardwareMap hardwareMap) {
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

    /** True when the current pipeline sees a target. */
    public boolean hasTarget() {
        return latest != null && latest.isValid();
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
     * Robot pose on the field from AprilTags (MegaTag1), or null with no target.
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
    }
}
