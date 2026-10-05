package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * SHELL: turn the robot to face the Limelight target (e.g. the goal's AprilTag) while the driver
 * keeps control of moving.
 * <p>
 * The logic is commented out until the robot and pipeline exist; for now it ends immediately so
 * binding it can't lock up the drivetrain. To finish it: uncomment, tune {@link #TURN_KP} and
 * {@link #TOLERANCE_DEG}, and delete the {@code return true} in {@link #isFinished()}.
 * It only requires drive: vision is read-only, so other commands can still use it.
 */
public class AimAtTargetCommand extends CommandBase {
    // Tune once the robot exists
    public static double TURN_KP = 0.02; // turn power per degree of error
    public static double TOLERANCE_DEG = 1.0;

    private final DriveSubsystem drive;
    private final VisionSubsystem vision;

    public AimAtTargetCommand(DriveSubsystem drive, VisionSubsystem vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // if (!vision.hasTarget()) return;
        // // tx is positive when the target is to the right; Pedro turns counterclockwise for positive
        // double turn = -vision.getTx() * TURN_KP;
        // drive.driveRobotCentric(0, 0, turn);
    }

    @Override
    public boolean isFinished() {
        // return vision.hasTarget() && Math.abs(vision.getTx()) < TOLERANCE_DEG;
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // drive.driveRobotCentric(0, 0, 0);
    }
}
