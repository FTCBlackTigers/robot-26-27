package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * Starts the flywheel and ends once it's at speed. The flywheel keeps spinning afterwards.
 * <p>
 * ShooterSubsystem is still a placeholder, so {@code isReady()} is always false and this never
 * ends by itself: always use it with {@code .withTimeout(ms)} until the shooter is real.
 */
public class SpinUpShooterCommand extends CommandBase {
    private final ShooterSubsystem shooter;

    public SpinUpShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.spinUp();
    }

    @Override
    public boolean isFinished() {
        return shooter.isReady();
    }
}
