package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.*;

public abstract class Subsystem {
    protected LinearOpMode opMode;
    protected boolean autonomous;
    protected String[] MOTOR_NAMES, SERVO_NAMES;
    public DcMotorEx[] motors;
    public Servo[] servos;

    public Subsystem(LinearOpMode opMode, final String[] MOTOR_NAMES, final String[] SERVO_NAMES, boolean autonomous) {
        this.opMode = opMode;
        this.MOTOR_NAMES = MOTOR_NAMES;
        this.SERVO_NAMES = SERVO_NAMES;
        this.autonomous = autonomous;
    }

    public abstract Map<String, Object> updateTelemetry();
}
