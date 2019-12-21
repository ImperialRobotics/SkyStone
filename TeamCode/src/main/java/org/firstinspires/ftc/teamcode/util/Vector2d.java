package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.MathUtil;

public class Vector2d {

    public final double x, y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d(Vector2d other) {
        this.x = other.x;
        this.y = other.y;
    }

    public Vector2d rotateBy(double angle) {
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
        double x  = this.x * cosA - this.y * sinA;
        double y = this.x * sinA + this.y * cosA;
        return new Vector2d(x,y);
    }
    public double angle() { return Math.atan2(y, x); }
    public double magnitude() {
        return Math.hypot(x,y);
    }

    public Vector2d plus(Vector2d other){
        return new Vector2d(x + other.x, x + other.x);
    }
    public Vector2d minus(Vector2d other){
        return new Vector2d(x - other.x, y - other.y);
    }
    public Vector2d times(double scalar) {
        return new Vector2d(x * scalar, y * scalar);
    }
    public Vector2d div(double scalar) {
        return new Vector2d(x / scalar, y / scalar);
    }
    public double dot(Vector2d other) {
        return x * other.x + y * other.y;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Vector2d)
            return MathUtil.approxEquals(((Vector2d) obj).x, x) && MathUtil.approxEquals(((Vector2d) obj).y, y);
        return false;
    }

    @Override
    public String toString() {
        return String.format("(%.3f, %.3f)", x, y);
    }
}