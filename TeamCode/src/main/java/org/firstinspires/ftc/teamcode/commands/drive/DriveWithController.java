package org.firstinspires.ftc.teamcode.commands.drive;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Field-centric driving from a gamepad. The drivetrain's default command in TeleOp, so it runs
 * whenever no other command (e.g. AimAtTarget) is using the drivetrain.
 */
public class DriveWithController extends CommandBase {
    private final Drivetrain drivetrain;
    private final GamepadEx controller;

    public DriveWithController(Drivetrain drivetrain, GamepadEx controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Pedro wants +forward, +left, +counterclockwise. GamepadEx.getLeftY() is already +up,
        // but getLeftX()/getRightX() are +right, so those two are negated.
        drivetrain.driveFieldCentric(
                controller.getLeftY(),
                -controller.getLeftX(),
                -controller.getRightX()
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
