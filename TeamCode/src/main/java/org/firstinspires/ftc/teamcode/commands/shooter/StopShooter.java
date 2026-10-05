package org.firstinspires.ftc.teamcode.commands.shooter;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/** Stops the flywheel and retracts the feeder. Ends immediately. */
public class StopShooter extends CommandBase {
    private final Shooter shooter;

    public StopShooter(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
