package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

public class LinearSlide extends Subsystem {

    private static final String[] MOTOR_NAMES = {"v_slide"};
    private static final String[] SERVO_NAMES = {"h_slide"};

    //presets
    private static final int[] PRESETS = {100, 200, 300, 400, 500};
    private int presetIndex;

    public Servo hSlide;

    //----------------------------------------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------------------------------------

    public LinearSlide(LinearOpMode opMode, boolean autonomous) {
        super(opMode, MOTOR_NAMES, SERVO_NAMES, autonomous);

        //configure motors
        motors = new DcMotorEx[MOTOR_NAMES.length];
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            motors[i] = opMode.hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i].setMode(STOP_AND_RESET_ENCODER);
            motors[i].setMode(RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(BRAKE);
        }

        //configure servos
        for(int i = 0; i < SERVO_NAMES.length; i++)
            servos[i] = opMode.hardwareMap.get(Servo.class, SERVO_NAMES[i]);

        //configure preset indices
        presetIndex = -1;

        opMode.telemetry.addData("Status", "Linear Slide instantiated");
        opMode.telemetry.update();
    }

    //----------------------------------------------------------------------------------------------
    // Presets
    //----------------------------------------------------------------------------------------------

    public void changeVerticalPreset(boolean increment) {
        motors[1].setTargetPosition(PRESETS[(increment ? ++presetIndex : --presetIndex) % PRESETS.length]);
        motors[1].setMode(RUN_TO_POSITION);
        motors[1].setPower(V_SLIDE_SPEED);
        while(motors[1].isBusy());
        motors[1].setPower(0);
        motors[1].setMode(RUN_USING_ENCODER);
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry
    //----------------------------------------------------------------------------------------------

    @Override
    public Map<String, Object> updateTelemetry() {
        Map<String, Object> telemetryData = new HashMap<>();
        for(int i = 0; i < motors.length; i++) {
            telemetryData.put(MOTOR_NAMES[i] + " encoder counts", motors[i].getCurrentPosition());
            telemetryData.put(MOTOR_NAMES[i] + " power", motors[i].getPower());
        }
        telemetryData.put("Vertical preset", PRESETS[presetIndex % PRESETS.length]);
        return telemetryData;
    }
}
