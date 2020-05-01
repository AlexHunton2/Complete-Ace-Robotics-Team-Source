package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Command
{
    protected String id;
    protected ElapsedTime commandTimer = new ElapsedTime();
    protected double timeoutInSeconds = FiniteStateMachine.COMMAND_TIMEOUT;
    protected CommandListener listener;

    protected Command()
    {
        id = java.util.UUID.randomUUID().toString();
    }

    protected Command(double timeoutinseconds)
    {
        id = java.util.UUID.randomUUID().toString();
        timeoutInSeconds = timeoutinseconds;
    }

    public String getID()
    {
        return id;
    }

    public void addCommandListener(CommandListener listener) { this.listener = listener;}
    public CommandListener getListener() { return listener; }

    public abstract void init();

    public abstract  void execute();

    public abstract boolean isFinished();

    public abstract void end();

    public void setTimeout(double timeoutInSeconds){
        this.timeoutInSeconds = timeoutInSeconds;
    }

    public String getName()
    {
        return getClass().getSimpleName();
    }

    public void resetTimer()
    {
        commandTimer.reset();
    }

    public double getSeconds()
    {
        return commandTimer.seconds();
    }

    public boolean isInTimeout()
    {
        return commandTimer.seconds() > timeoutInSeconds;
    }

    public String toString()
    {
        return getName();
    }
}
