package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.*;

public abstract class Subsystem {
    protected LinearOpMode opMode;
    protected boolean autonomous;
    protected String[] MOTOR_NAMES;
    public DcMotorEx[] motors;

    public Subsystem(LinearOpMode opMode, final String[] MOTOR_NAMES, boolean autonomous) {
        this.opMode = opMode;
        this.MOTOR_NAMES = MOTOR_NAMES;
        this.autonomous = autonomous;
    }

    public abstract Map<String, Object> updateTelemetry();
}
