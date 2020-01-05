package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.PIDFController;

import java.util.HashMap;
import java.util.Map;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

public class Intake extends Subsystem {
    private static final String[] MOTOR_NAMES = {"right_intake", "left_intake"};
    IntakeState state;

    //----------------------------------------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------------------------------------

    public Intake(LinearOpMode opMode, boolean autonomous) {
        super(opMode, MOTOR_NAMES, null, autonomous);

        //configure motors
        motors = new DcMotorEx[MOTOR_NAMES.length];
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            motors[i] = opMode.hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i].setMode(STOP_AND_RESET_ENCODER);
            motors[i].setMode(RUN_USING_ENCODER);
        }

        state = IntakeState.INTAKE;

        opMode.telemetry.addData("Status", "Intake instantiated");
        opMode.telemetry.update();
    }

    @Override
    public Map<String, Object> updateTelemetry() {
        Map<String, Object> telemetryData = new HashMap<>();

        for(int i = 0; i < motors.length; i++) {
            telemetryData.put(MOTOR_NAMES[i] + " encoder position", motors[i].getCurrentPosition());
            telemetryData.put(MOTOR_NAMES[i] + " power", motors[i].getPower());
        }
        telemetryData.put("intake state", state);

        return telemetryData;
    }

    public void incrementState() {
        if(state.equals(IntakeState.INTAKE))
            state = IntakeState.SPIT;
        else if(state.equals(IntakeState.SPIT))
            state = IntakeState.DISABLED;
        else
            state = IntakeState.INTAKE;
    }

    public void handleState() {
        switch(state) {
            case INTAKE:
                setPowers(INTAKE_SPEED, -INTAKE_SPEED);
                break;
            case SPIT:
                setPowers(-INTAKE_SPEED, INTAKE_SPEED);
                break;
            case DISABLED:
                for(DcMotorEx motor: motors)
                    motor.setPower(0);
                break;
        }
    }

    private void setPowers(double leftPower, double rightPower) {
        motors[0].setPower(leftPower);
        motors[1].setPower(rightPower);
    }

    public enum IntakeState {
        INTAKE, SPIT, DISABLED
    }
}
