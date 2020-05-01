package org.firstinspires.ftc.teamcode.statemachine;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.teamcode.utils.Dashboard;

import java.util.ArrayList;

public class FiniteStateMachine extends StateMachine
{
    // all commands need a timeout
    public final static double COMMAND_TIMEOUT = 10.0;

    private ArrayList<Command> commands = new ArrayList<>();
    private int currentCommandIndex = -1;

    private ElapsedTime globalTimer = new ElapsedTime();


    public FiniteStateMachine()
    {
        Dashboard.trace(getClass(), "Constructor");
        globalTimer.reset();
    }

    public void resetTime()
    {
        globalTimer.reset();
    }

    public void addCommand(Command command)
    {
        Dashboard.trace(getClass(), "Adding command " + command.getClass().getSimpleName());
        commands.add(command);
    }


    public void addCommand(final double delay1, Command command1, final double delay2, Command command2)
    {
        command1.addCommandListener(new CommandListener() {
            @Override
            public boolean canExecute(CommandGroup context) {
                Dashboard.debug("cmd1 getSeconds="+ context.getSeconds());
                return context.getSeconds() > delay1;
            }
        });
        command2.addCommandListener(new CommandListener() {
            @Override
            public boolean canExecute(CommandGroup context) {
                Dashboard.debug("cmd2 getSeconds="+ context.getSeconds());
                return context.getSeconds() > delay2;
            }
        });
        commands.add(new CommandGroup(command1, command2));
    }


    /** execute the current command until the command is finished,
     * then proceed to the next command */
    public void run()
    {
        if (currentCommandIndex == -1) // only for the first command
        {
            next();
        }

        Command cmd = getCurrentCommand();
        if (cmd == null)
        {
            Dashboard.trace(getClass(), "no more commands");
            return;
        }

        if(cmd.isInTimeout())
        {
            Dashboard.trace(getClass(), "COMMAND " + getCurrentCommandName() + " timeout");
            next();
            return;
        }

        if (cmd.isFinished())
        {
            Dashboard.trace(getClass(), "COMMAND " + getCurrentCommandName() + " finished");
            next();
            return;
        }

        Dashboard.trace(getClass(), "COMMAND " + getCurrentCommandName() + " running, elapsed "+cmd.getSeconds()+"s, " + cmd.toString());
        Dashboard.addData("FSM Command: ", getCurrentCommandName() );
        cmd.execute();
    }

//    public void addState(Command state, Command next)
//    {
//        commands.put(state, next);
//
//        if(currentState == null)
//        {
//            initCommand = state;
//            currentState = state;
////            globalTimer.start();
//        }
//    }



    private  Command getCurrentCommand()
    {
        return (currentCommandIndex < 0 || currentCommandIndex >= commands.size()) ? null : commands.get(currentCommandIndex);
    }

    private String getCurrentCommandName()
    {
        Command cmd = getCurrentCommand();
        return (cmd == null) ? "" : cmd.getName();
    }

//    public void resetInstance()
//    {
//        currentState = initCommand;
//    }


    private void next()
    {

        Command cmd;

        // end the current command
        cmd = getCurrentCommand();
        if(cmd != null) {
            Dashboard.trace(getClass(), "COMMAND "+cmd.getName()+".end()");
            cmd.end();
        }

        // move to the next command (can be null)
        currentCommandIndex++;

        // setTelemetry the new command
        cmd = getCurrentCommand();
        if(cmd != null) {
            Dashboard.trace(getClass(), "================= NEW COMMAND ====================");
            Dashboard.trace(getClass(), "=    " + cmd.getName());
            Dashboard.trace(getClass(), "==================================================");
            Dashboard.trace(getClass(), "COMMAND "+cmd.getName()+".init()");
            cmd.init();
            Dashboard.trace(getClass(), "COMMAND "+cmd.getName()+".resetTimer()");
            cmd.resetTimer();
        }
    }


    public boolean hasGlobalDurationElapsed(double maxduration)
    {
        return globalTimer.seconds() > maxduration;
    }

}
