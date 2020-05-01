package org.firstinspires.ftc.teamcode.utils;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Dashboard {

    private static org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

    public static void setTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry t)
    {
        telemetry = t;
    }

    public static void addData(String caption, Object value )
    {
        if(telemetry == null) return;
        telemetry.addData(caption, value);
    }


    public static void banner()
    {
        Log.e("[ACE TRACE]", "-");
        Log.e("[ACE TRACE]", "-");
        Log.e("[ACE TRACE]", "-");
        Log.e("[ACE TRACE]", "------------- [ACE IS STARTING!]------------");
        Log.e("[ACE TRACE]", "                                            ");
        Log.e("[ACE TRACE]", "   #####  #######    #    ######  #######   ");
        Log.e("[ACE TRACE]", "  #     #    #      # #   #     #    #      ");
        Log.e("[ACE TRACE]", "  #          #     #   #  #     #    #      ");
        Log.e("[ACE TRACE]", "   #####     #    #     # ######     #      ");
        Log.e("[ACE TRACE]", "        #    #    ####### #   #      #      ");
        Log.e("[ACE TRACE]", "  #     #    #    #     # #    #     #      ");
        Log.e("[ACE TRACE]", "   #####     #    #     # #     #    #      ");
        Log.e("[ACE TRACE]", "                                            ");
        Log.e("[ACE TRACE]", "------------- [ACE IS STARTING!]------------");
        Log.e("[ACE TRACE]", "-");
    }

    public static void debug(String value )
    {
        Log.e("[ACE DEBUG]", "@@@@@@@@@@@@@@ " +value);
    }

    public static void trace(Class clasz, String value )
    {
        String name = clasz!=null ? clasz.getSimpleName() : "UnknownClassName";
        Log.d("[ACE TRACE]", "["+name+"] " +value);
    }
}
