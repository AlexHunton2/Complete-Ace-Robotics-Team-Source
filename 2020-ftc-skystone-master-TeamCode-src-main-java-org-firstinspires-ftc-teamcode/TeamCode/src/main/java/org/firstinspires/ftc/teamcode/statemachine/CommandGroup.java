package org.firstinspires.ftc.teamcode.statemachine;

import org.firstinspires.ftc.teamcode.utils.Dashboard;

public class CommandGroup extends Command {

    private Command command1, command2;

    public CommandGroup(Command command1, Command command2) {
        id = java.util.UUID.randomUUID().toString();
        this.command1 = command1;
        this.command2 = command2;
    }

    @Override
    public void resetTimer()
    {
        command1.resetTimer();
        command2.resetTimer();
    }

    @Override
    public boolean isInTimeout()
    {
        return command1.isInTimeout() && command2.isInTimeout();
    }

    @Override
    public void init()
    {
        command1.init();
        command2.init();
    }

    @Override
    public void execute()
    {
        if(command1.getListener()==null || (command1.getListener()!=null && command1.getListener().canExecute(this))) command1.execute();
        if(command2.getListener()==null || (command2.getListener()!=null && command2.getListener().canExecute(this))) command2.execute();
    }

    @Override
    public boolean isFinished()
    {
        return (command1.isInTimeout() || command1.isFinished()) && (command2.isInTimeout() || command2.isFinished());
    }

    @Override
    public void end() {
        command1.end();
        command2.end();
    }

}

