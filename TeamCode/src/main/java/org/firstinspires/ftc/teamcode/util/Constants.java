package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Constants {

    //----------------------------------------------------------------------------------------------
    // Drive Constants
    //----------------------------------------------------------------------------------------------

    public static double INTAKE_SPEED = 1.0;
    public static double V_SLIDE_SPEED = 1.0;
    public static double H_SLIDE_SPEED = 1.0;

    public static PIDFCoefficients DRIVE_PID_COEFFICIENTS = new PIDFCoefficients(0.15, 0.0, 0.05);
    public static PIDFCoefficients STRAFE_PID_COEFFICIENTS = new PIDFCoefficients(0.1, 0.0, 0.05);
    public static PIDFCoefficients ROTATE_PID_COEFFICIENTS = new PIDFCoefficients(0.1, 0.0, 0.05);

    public static double EPSILON = 0.001;

    //----------------------------------------------------------------------------------------------
    // Tele-Op Constants
    //----------------------------------------------------------------------------------------------

    public static double TRIGGER_DEADZONE = 0.2;
    public static double JOYSTICK_DEADZONE = 0.05;
    public static double SCALING = 1.145;

    //----------------------------------------------------------------------------------------------
    // Physical Constants
    //----------------------------------------------------------------------------------------------

    public static double WHEEL_DIAMETER = 3.543;
    public static double TICKS_PER_REVOLUTION = MotorConfigurationType.getMotorType(NeveRest40Gearmotor.class).getTicksPerRev();
    public static double GEAR_RATIO = 0.4;
    public static double TICKS_PER_INCH = (TICKS_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);
}