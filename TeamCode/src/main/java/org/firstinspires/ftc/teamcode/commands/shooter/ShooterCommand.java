package org.firstinspires.ftc.teamcode.commands.shooter;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/** Runs the shooter while the button is held (bind with whenHeld) and stops it on release. */
public class ShooterCommand extends CommandBase {
    private final Shooter shooter;

    public ShooterCommand(Shooter shooter)
    {
        this.shooter = shooter;
        addRequirements(shooter);
    }


    @Override
    public void initialize() {
        shooter.spinUp();
    }

    // Runs once when the command ends or is interrupted (button released)
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
