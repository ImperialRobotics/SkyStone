package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private static final String[] MOTOR_NAMES = {"fr", "fl", "br", "bl"};

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
        double fr = y - x - rot;
        double fl = y + x + rot;
        double br = y + x - rot;
        double bl = y - x + rot;
        double max = MathUtil.max(fr, fl, br, bl);
        if(max >= 1.0) {
            fr /= max;
            fl /= max;
            br /= max;
            bl /= max;
        }
        setPowers(fr, fl, br, bl);
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
        input = input.rotateBy(-getAngle());

        double theta = input.angle();
        double magnitude = input.magnitude();
        setPowers(
                magnitude * Math.sin(theta - Math.PI / 4) - rot,
                magnitude * Math.sin(theta + Math.PI / 4) + rot,
                magnitude * Math.sin(theta + Math.PI / 4) - rot,
                magnitude * Math.sin(theta - Math.PI / 4) + rot
        );
    }

    //----------------------------------------------------------------------------------------------
    // Helper Methods
    //----------------------------------------------------------------------------------------------

    private void setTargetPosition(int targetPosition) {
        setTargetPositions(targetPosition, targetPosition, targetPosition, targetPosition);
    }

    private void setTargetPositions(int... targetPositions) {
        for(int i = 0; i < targetPositions.length; i++)
            motors[i].setTargetPosition(targetPositions[i]);
    }

    private void setPower(double power) {
        setPowers(power, power, power, power);
    }

    private void setPowers(double... powers) {
        for(int i = 0; i < powers.length; i++)
            motors[i].setPower(powers[i]);
    }

    private void setMode(DcMotorEx.RunMode runMode) {
        for(DcMotorEx motor: motors)
            motor.setMode(runMode);
    }

    private boolean isBusy() {
        boolean isBusy = false;
        for(DcMotorEx motor: motors)
            isBusy = isBusy() || motor.isBusy();
        return isBusy;
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
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    //----------------------------------------------------------------------------------------------
    // Autonomous Non-PID Methods
    //----------------------------------------------------------------------------------------------

    /**
     * drives a given number of inches
     * @param inches positive when driving forwards, negative when driving backwards
     */
    public void drive(double inches) {
        int ticks = MathUtil.ticks(inches);

        opMode.telemetry.addData("Driving", "ticks: " + ticks);
        opMode.telemetry.update();

        for(DcMotorEx motor: motors)
            motor.setTargetPosition(motor.getCurrentPosition() + ticks);
        setMode(RUN_TO_POSITION);
        setPower(DRIVE_SPEED);

        long startTime = System.currentTimeMillis();
        while(opMode.opModeIsActive() && isBusy() && System.currentTimeMillis() - startTime < AUTONOMOUS_DRIVE_TIMEOUT) {
            opMode.telemetry.addData("heading", Math.toDegrees(getAngle()));
            opMode.telemetry.update();
        }
        setPower(0);

        opMode.sleep(SLEEP_TIME);
        setMode(RUN_USING_ENCODER);

        opMode.telemetry.addData("Finished Driving", "ticks: " + ticks);
    }

    /**
     * strafes a given number of inches
     * @param inches positive when strafing right, negative when strafing left
     */
    public void strafe(double inches) {
        int ticks = MathUtil.ticks(inches);

        opMode.telemetry.addData("Strafing", "ticks: " + ticks);
        opMode.telemetry.update();

        for(int i = 0; i < motors.length; i++) {
            motors[i].setTargetPosition(motors[i].getCurrentPosition() + (i == 1 || i == 2 ? ticks : -ticks));
        }

        setMode(RUN_TO_POSITION);
        setPower(STRAFE_SPEED);

        long startTime = System.currentTimeMillis();
        while(opMode.opModeIsActive() && motors[2].isBusy() && System.currentTimeMillis() - startTime < AUTONOMOUS_STRAFE_TIMEOUT) {
            opMode.telemetry.addData("heading", Math.toDegrees(getAngle()));
            opMode.telemetry.update();
        }
        motors[2].setPower(0);
        motors[2].setMode(RUN_WITHOUT_ENCODER);

        opMode.telemetry.addData("Finished Strafing", "ticks: " + ticks);
        opMode.telemetry.update();

        opMode.sleep(SLEEP_TIME);
    }

    /**
     * rotates a given number of radians relative to current heading
     * @param radians negative when rotating clockwise, positive when rotating counterclockwise
     */
    public void rotate(double radians) {
        resetAngle();

        opMode.telemetry.addData("Rotating", "degrees: " + Math.toDegrees(radians));
        opMode.telemetry.update();

        setMode(RUN_USING_ENCODER);

        radians = MathUtil.wrapAngle(radians);

        for(int i = 0; i < motors.length; i++) {
            if(radians > 0)
                motors[i].setPower(i == 0 || i == 2 ? -ROTATE_SPEED : ROTATE_SPEED);
            else
                motors[i].setPower(i == 0 || i == 2 ? ROTATE_SPEED : -ROTATE_SPEED);
        }

        if(radians == 0)
            return;

        long startTime = System.currentTimeMillis();

        if (radians < 0) {
            while (opMode.opModeIsActive() && getAngle() == 0 && System.currentTimeMillis() - startTime < AUTONOMOUS_ROTATE_TIMEOUT);

            while (opMode.opModeIsActive() && getAngle() > radians && System.currentTimeMillis() - startTime < AUTONOMOUS_ROTATE_TIMEOUT);
        }
        else
            while (opMode.opModeIsActive() && getAngle() < radians && System.currentTimeMillis() - startTime < AUTONOMOUS_ROTATE_TIMEOUT);

        setPower(0);

        opMode.sleep(SLEEP_TIME);

        resetAngle();

        opMode.telemetry.addData("Finished Rotating", "degrees: " + radians);
        opMode.telemetry.update();
    }

    //----------------------------------------------------------------------------------------------
    // Autonomous PID Methods
    //----------------------------------------------------------------------------------------------

    /**
     * drives a given number of inches, keeping a constant heading through use of a PID
     * controller
     * @param inches positive when driving forwards, negative when driving backwards
     */
    public void drivePID(double inches) {
        int ticks = MathUtil.ticks(inches);

        opMode.telemetry.addData("Driving PID", "ticks: " + ticks);
        opMode.telemetry.update();

        for(DcMotorEx motor : motors)
            motor.setTargetPosition(motor.getCurrentPosition() + ticks);

        setMode(RUN_TO_POSITION);

        resetAngle();

        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(-DRIVE_SPEED, DRIVE_SPEED);

        long startTime = System.currentTimeMillis();

        while(opMode.opModeIsActive() && motors[0].isBusy() && motors[1].isBusy() &&
                System.currentTimeMillis() - startTime < AUTONOMOUS_DRIVE_TIMEOUT) {
            double steer = pidDrive.update(getAngle(), imu.getVelocity().zVeloc, imu.getAcceleration().zAccel);

            // if driving in reverse, the motor correction also needs to be reversed
            if (inches < 0)
                steer *= -1.0;

            double leftSpeed = DRIVE_SPEED - steer;
            double rightSpeed = DRIVE_SPEED + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            setPowers(rightSpeed, leftSpeed, rightSpeed, leftSpeed);

            opMode.telemetry.addData("heading", Math.toDegrees(globalAngle));
            opMode.telemetry.addData("ticks traveled", "left: %d, right: %d", motors[0].getCurrentPosition(), motors[1].getCurrentPosition());
            opMode.telemetry.update();
        }

        setPower(0);
        setMode(RUN_USING_ENCODER);

        opMode.telemetry.addData("Finished Driving PID", "ticks: " + ticks);
        opMode.telemetry.update();

        opMode.sleep(SLEEP_TIME);
    }

    /**
     * performs one cycle of closed loop heading control for the rotatePID() method
     * @return true if error of heading is within tolerable range, false if not
     */
    private boolean onHeading() {
        boolean onTarget = pidRotate.onTarget();

        if(onTarget) {
            setPower(0);
        } else {
            double steer = pidRotate.update(getAngle(), imu.getVelocity().zVeloc, imu.getAcceleration().zAccel);
            setPowers(ROTATE_SPEED * steer, -ROTATE_SPEED * steer, ROTATE_SPEED * steer, -ROTATE_SPEED * steer);
        }

        return onTarget;
    }

    /**
     * rotates a given number of degrees, doing so by use of a PID controller
     * @param degrees negative when rotating clockwise, positive when rotating counterclockwise
     */
    public void rotatePID(double degrees) {

        opMode.telemetry.addData("Rotating PID", "degrees: " + degrees);
        opMode.telemetry.update();


        resetAngle();
        if (Math.abs(degrees) > 360)
            degrees = (int) Math.copySign(360, degrees);

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, ROTATE_SPEED);
        pidRotate.setTolerance(.01);

        double startTime = System.currentTimeMillis();

        while(opMode.opModeIsActive() && !onHeading() && System.currentTimeMillis() - startTime < AUTONOMOUS_ROTATE_TIMEOUT) {
            opMode.telemetry.addData("Heading", getAngle());
            opMode.telemetry.update();
        }

        opMode.telemetry.addData("Finished Rotating PID", "degrees: " + degrees);
        opMode.telemetry.update();

        opMode.sleep(SLEEP_TIME);
    }
}
