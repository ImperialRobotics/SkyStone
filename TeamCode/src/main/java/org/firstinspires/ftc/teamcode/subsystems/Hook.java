package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

public class Hook extends Subsystem {

    public static final String[] SERVO_NAMES = {"l_hook, r_hook"};

    public Hook(LinearOpMode opMode, boolean autonomous) {
        super(opMode, null, SERVO_NAMES, autonomous);
        servos = new Servo[SERVO_NAMES.length];
        for(int i = 0; i < SERVO_NAMES.length; i++)
            servos[i] = opMode.hardwareMap.get(Servo.class, SERVO_NAMES[i]);
    }

    public void deploy() {
        servos[0].setPosition(1.0);
        servos[1].setPosition(1.0);
    }

    public void retract() {
        servos[0].setPosition(0.0);
        servos[1].setPosition(0.0);
    }

    @Override
    public Map<String, Object> updateTelemetry() {
        Map<String, Object> telemetryData = new HashMap<>();
        for(int i = 0; i < SERVO_NAMES.length; i++)
            telemetryData.put(SERVO_NAMES[i] + " position", servos[i].getPosition());
        return telemetryData;
    }
}
