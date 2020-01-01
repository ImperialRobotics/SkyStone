package org.firstinspires.ftc.teamcode.util;

import java.util.Vector;

public class Pose2d {
    public double x, y, heading;

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d(Vector2d pos, double heading) {
        this(pos.x, pos.y, heading);
    }

    public Vector2d vec() {
        return new Vector2d(x, y);
    }

    public Pose2d plus(Pose2d other) {
        return new Pose2d(x + other.x, y + other.y, heading + other.heading);
    }
    public Pose2d minus(Pose2d other) {
        return new Pose2d(x - other.x, y - other.y, heading - other.heading);
    }
    public Pose2d times(double scalar) {
        return new Pose2d(x * scalar, y * scalar, heading * scalar);
    }
    public Pose2d div(double scalar) {
        return new Pose2d(x / scalar, y / scalar, heading / scalar);
    }
}
