package org.firstinspires.ftc.teamcode.utils;

public class Utils
{
    public static double speedLimiter(double speed)
    {
        return Math.max(-1d, Math.min(1d,speed));
    }

    // account for negative values and when min>max
    public static boolean inRange(double value, double min, double max)
    {
        return value < Math.max(min,max) && value > Math.min(min,max);

    }

    public static boolean inRangePercent(double value, double targetValue, double percent)
    {
        return inRange(value, targetValue*(1-percent), targetValue*(1+percent));
    }

    public static String toNearestHundredth(double number)
    {
        return String.format("%.2f", number);
    }


}
