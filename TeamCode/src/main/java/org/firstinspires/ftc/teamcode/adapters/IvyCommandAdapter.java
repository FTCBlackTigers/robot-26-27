package org.firstinspires.ftc.teamcode.adapters;

import com.pedropathing.ivy.Command;

import com.seattlesolvers.solverslib.command.CommandBase;

public class IvyCommandAdapter {

    public static Command fromSolversLib(CommandBase command) {

        return Command.build()
                .setStart(command::initialize)
                .setExecute(command::execute)
                .setDone(command::isFinished)
                .setEnd(endCondition -> command.end(false));
    }
}