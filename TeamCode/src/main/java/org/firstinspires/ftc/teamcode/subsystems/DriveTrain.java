package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.Vector2d;

import java.util.HashMap;
import java.util.Map;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

public class DriveTrain extends Subsystem {
    private static final String[] MOTOR_NAMES = {"rf", "lf", "rb", "lb"};

    //heading related PIVs
    public BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;

    //pid controllers
    private PIDFController pidDrive, pidStrafe, pidRotate;

    //----------------------------------------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------------------------------------

    public DriveTrain(LinearOpMode opMode, boolean autonomous) {
        super(opMode, MOTOR_NAMES, null, autonomous);

        //configure motors
        motors = new DcMotorEx[MOTOR_NAMES.length];
        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            motors[i] = opMode.hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i].setMode(STOP_AND_RESET_ENCODER);
            motors[i].setMode(RUN_USING_ENCODER);
            motors[i].setDirection(i == 1 || i == 3 ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        }

        //initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        opMode.telemetry.addData("Status", "calibrating imu");
        opMode.telemetry.update();

        while (!opMode.isStopRequested() && !imu.isGyroCalibrated()) {
            opMode.sleep(50);
            opMode.idle();
        }

        opMode.telemetry.addData("Status", "calibrated imu");
        opMode.telemetry.update();

        //initialize PIDControllers
        pidDrive = new PIDFController(DRIVE_PID_COEFFICIENTS);
        pidStrafe = new PIDFController(STRAFE_PID_COEFFICIENTS);
        pidRotate = new PIDFController(ROTATE_PID_COEFFICIENTS);

        opMode.telemetry.addData("Status", "DriveTrain instantiated");
        opMode.telemetry.update();
    }

    public void driveRobotCentric(double x, double y, double rot) {
        x = Range.clip(x, -1, 1);
        y = Range.clip(y, -1, 1);
        rot = Range.clip(rot, -1, 1);

        double theta = Math.atan(y/x) * 0.5;
        double r = Math.hypot(x, y);
        r = r * (100 / 127) - rot;
        double xComp = r * Math.cos(theta);
        double yComp = r * Math.sin(theta);

        setPowers(rot + xComp, rot - xComp, rot - yComp, rot + yComp);
    }

    /**
     * Drives the robot from the perspective of the driver. No matter the orientation of the
     * robot, pushing forward on the drive stick will always drive the robot away
     * from the driver.
     *
     * @param x the horizontal speed of the robot, derived from input
     * @param y the vertical speed of the robot, derived from input
     * @param rot the turn speed of the robot, derived from input
     */
    public void driveFieldCentric(double x, double y, double rot) {
        x = Range.clip(x, -1, 1);
        y = Range.clip(y, -1, 1);
        rot = Range.clip(rot, -1, 1);

        Vector2d input = new Vector2d(x, y);
        input = input.rotateBy(-rot);

        double theta = input.angle();
        double magnitude = input.magnitude();
        setPowers(
                magnitude * Math.sin(theta - Math.PI / 4) - rot,
                magnitude * Math.sin(theta + Math.PI / 4) + rot,
                magnitude * Math.sin(theta + Math.PI / 4) - rot,
                magnitude * Math.sin(theta - Math.PI / 4) + rot
        );
    }


    public void setPowers(double rf, double lf, double rb, double lb) {
        motors[0].setPower(rf);
        motors[1].setPower(lf);
        motors[2].setPower(rb);
        motors[3].setPower(lb);
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry
    //----------------------------------------------------------------------------------------------

    @Override
    public Map<String, Object> updateTelemetry() {
        Map<String, Object> telemetryData = new HashMap<>();

        telemetryData.put("heading", globalAngle);
        for(int i = 0; i < motors.length; i++) {
            telemetryData.put(MOTOR_NAMES[i] + " encoder counts", motors[i].getCurrentPosition());
            telemetryData.put(MOTOR_NAMES[i] + " power", motors[i].getPower());
        }
        telemetryData.put("pidDrive", pidDrive.telemetryData());
        telemetryData.put("pidStrafe", pidStrafe.telemetryData());
        telemetryData.put("pidRotate", pidRotate.telemetryData());

        return telemetryData;
    }

    //----------------------------------------------------------------------------------------------
    // Autonomous Helper Methods
    //----------------------------------------------------------------------------------------------

    /**
     * @return returns + when rotating counter clockwise (left) and - when rotating clockwise (right)
     */
    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = MathUtil.wrapAngle(angles.firstAngle - lastAngles.firstAngle);
        lastAngles = angles;
        return globalAngle;
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
}
