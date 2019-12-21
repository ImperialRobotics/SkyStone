package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.Constants.*;

public class MathUtil {
    public static int ticks(double inches) {
        return (int) Math.round(inches * TICKS_PER_INCH);
    }

    /**
     * makes sure an angle is within the range -pi to pi radians
     * @param angle angle to be wrapped
     * @return angle wrapped to between -pi and pi radians (-180 to 180 degrees)
     */
    public static double wrapAngle(double angle) {
        while(angle < -Math.PI)
            angle += 2 * Math.PI;
        while(angle > Math.PI)
            angle -= 2 * Math.PI;
        return angle;
    }

    public static double deadZone(double x, double threshold) {
        return Math.abs(x) < threshold ? 0 : x;
    }

    public static double scale(double x) {
        return x == 0.0 ? 0 : Math.pow(x, 3) / Math.pow(Math.abs(x), SCALING);
    }

    public static boolean approxEquals(double x, double y) {
        return Math.abs(x - y) < EPSILON;
    }

    public static double max(double... values) {
        double max = Double.NEGATIVE_INFINITY;
        for(double value : values)
            if(value > max)
                max = value;
        return max;
    }
}
