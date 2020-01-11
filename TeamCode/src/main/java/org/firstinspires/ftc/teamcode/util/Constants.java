package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.opencv.core.Scalar;

public class Constants {

    //----------------------------------------------------------------------------------------------
    // Drive Constants
    //----------------------------------------------------------------------------------------------

    public static double DRIVE_SPEED = 0.75;
    public static double STRAFE_SPEED = 0.75;
    public static double ROTATE_SPEED = 0.4;

    public static PIDFCoefficients DRIVE_PID_COEFFICIENTS = new PIDFCoefficients(0.15, 0.0, 0.05);
    public static PIDFCoefficients STRAFE_PID_COEFFICIENTS = new PIDFCoefficients(0.1, 0.0, 0.05);
    public static PIDFCoefficients ROTATE_PID_COEFFICIENTS = new PIDFCoefficients(0.1, 0.0, 0.05);

    public static int AUTONOMOUS_DRIVE_TIMEOUT = 2500;
    public static int AUTONOMOUS_STRAFE_TIMEOUT = 2500;
    public static int AUTONOMOUS_ROTATE_TIMEOUT = 2500;

    public static int SLEEP_TIME = 0;

    public static double EPSILON = 0.001;

    //----------------------------------------------------------------------------------------------
    // Tele-Op Constants
    //----------------------------------------------------------------------------------------------

    public static double V_SLIDE_SPEED = 1.0;
    public static double H_SLIDE_SPEED = 0.5;
    public static double HOOK_INCREMENT = 0.01;
    public static double GRIPPER_INCREMENT = 0.01;

    public static double TRIGGER_DEADZONE = 0.2;
    public static double JOYSTICK_DEADZONE = 0.05;

    public static double SCALING = 1.145;

    public static int V_SLIDE_MAX_TICKS = 3125;

    public static double SMOOTHING_FACTOR = 0.97;

//    public static double GRIPPER_GRIP_POSITION = 0.0;
//    public static double GRIPPER_RESTING_POSITION = 0.3;

    //----------------------------------------------------------------------------------------------
    // Physical Constants
    //----------------------------------------------------------------------------------------------

    public static double WHEEL_DIAMETER = 3.543;
    public static double TICKS_PER_REVOLUTION = MotorConfigurationType.getMotorType(NeveRest40Gearmotor.class).getTicksPerRev();
    public static double GEAR_RATIO = 1.0;
    public static double TICKS_PER_INCH = (TICKS_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);

    //----------------------------------------------------------------------------------------------
    // Vision Constants
    //----------------------------------------------------------------------------------------------

    public static final Scalar MIN_HSV = new Scalar(0, 140.7, 100.5);
    public static final Scalar MAX_HSV = new Scalar(39.3, 255.0, 255.0);
    
    public static final int ERODE_ITERATIONS = 10;
    public static final int DILATE_ITERATIONS = 20;
    
    //vary based on distance from blocks
    public static final double MIN_CONTOUR_AREA = 120.0;
    public static final double MIN_CONTOUR_PERIMETER = 140.0;
    public static final double MIN_CONTOUR_WIDTH = 100.0;
    public static final double MIN_CONTOUR_HEIGHT = 0.0;
    
    public static final double CB_MIN = 105;
    public static final double CB_MAX = 140;
    
    //vary based on webcam placement
    public static final int MAX_VUMARK_VALUE = 80; // used to be 150
    public static final int VALLEY_LENGTH = 28;
}