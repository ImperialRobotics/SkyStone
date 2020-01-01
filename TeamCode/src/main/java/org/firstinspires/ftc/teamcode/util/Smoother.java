package org.firstinspires.ftc.teamcode.util;

public class Smoother {
    private Pose2d lastCommand;
    private long lastTime;
    public double exponent;
    private Mode mode;

    public Smoother(Mode mode, double exponent) {
        lastCommand = new Pose2d(0, 0, 0);
        lastTime = 0;
        this.exponent = exponent;
        this.mode = mode;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public Pose2d transform(Pose2d original) {
        if (lastTime == 0) {
            lastTime = System.currentTimeMillis();
            return lastCommand;
        }

        long now = System.currentTimeMillis();
        long dt = now - lastTime;
        lastTime = now;

        double r = original.vec().magnitude();
        double omega = original.heading;
        double sigOmega = Math.signum(omega);
        omega = Math.abs(omega);

        switch (mode) {
            case LINEAR: break;
            case EXPONENTIAL:
                r = Math.pow(r, exponent);
                omega = Math.pow(omega, exponent);
                break;
        }

        Pose2d command = new Pose2d(original.vec().times(r / original.vec().magnitude()), omega * sigOmega);
        if (r == 0) command = new Pose2d(0, 0, omega * sigOmega);
        return command;
    }
    
    public enum Mode {
        LINEAR, EXPONENTIAL;
    }
}