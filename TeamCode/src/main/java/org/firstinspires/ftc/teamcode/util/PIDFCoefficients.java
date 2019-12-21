package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PIDFCoefficients {
    public double kP, kI, kD;
    public double kV, kA, kStatic;
    public kF kF;

    public PIDFCoefficients(double kP, double kI, double kD,
                            double kV, double kA, double kStatic, kF kF) {
        this(kP, kI, kD);
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.kF = kF;
    }

    public PIDFCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        kV = 0;
        kA = 0;
        kStatic = 0;
        kF = x -> 0;
    }

    public String toString() {
        return String.format("kP: %.3f, kI: %.3f, kD: %.2f, kV: %.2f, kA: %.2f, kStatic: %.2f", kP, kI, kD, kV, kA, kStatic);
    }
}

interface kF {
    double update(double x);
}
