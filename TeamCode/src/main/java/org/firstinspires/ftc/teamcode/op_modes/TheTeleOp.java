package org.firstinspires.ftc.teamcode.op_modes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.StickyGamepad;

import static org.firstinspires.ftc.teamcode.util.Constants.*;


/** Controls:
 * left_stick_x - horizontal movement
 * left_stick_y - vertical movement
 * right_stick_x - rotate
 *
 *  b - loosen gripper
 *  x - tighten gripper
 *  dpad_up - retract hook
 *  dpad_down - lower hook
 *  right_bumper - extend vertical linear slide
 *  left_bumper - retract vertical linear slide
 *  right_trigger - extend horizontal linear slide
 *  left_trigger - retract horizontal linear slide
 *
 *  y - toggle smoothing
 *  a - increase scaling
 *  guide - emergency stop
 */

@TeleOp(name="the teleop")
public class TheTeleOp extends LinearOpMode {

    private Robot robot;
    private boolean isRobotCentric;
    private StickyGamepad stickyGamepad1;
    private boolean isStopped, isSmoothed, isGripped;
    private double scaling;

    @Override
    public void runOpMode() {

        robot = new Robot(this, false, false);
        stickyGamepad1 = new StickyGamepad(gamepad1);
        isRobotCentric = true;
        isStopped = false;
        isSmoothed = false;
        isGripped = false;
        scaling = 1.0;

        waitForStart();
        while(opModeIsActive() && !isStopped) {


            if(stickyGamepad1.y) {
                isSmoothed = !isSmoothed;
                robot.driveTrain.resetSmoothers();
            }
            if(stickyGamepad1.a) {
                scaling = (scaling == 1.0 ? 0.25 : scaling + 0.25);
                robot.driveTrain.resetSmoothers();
            }

            if(gamepad1.right_bumper && robot.linearSlide.motors[0].getCurrentPosition() <= V_SLIDE_MAX_TICKS)
                robot.linearSlide.motors[0].setPower(V_SLIDE_SPEED);
            else if(gamepad1.left_bumper)
                robot.linearSlide.motors[0].setPower(-V_SLIDE_SPEED);
            else
                robot.linearSlide.motors[0].setPower(0);

            if(MathUtil.deadZone(gamepad1.right_trigger, TRIGGER_DEADZONE) > 0)
                robot.linearSlide.motors[1].setPower(H_SLIDE_SPEED);
            else if(MathUtil.deadZone(gamepad1.left_trigger, TRIGGER_DEADZONE) > 0)
                robot.linearSlide.motors[1].setPower(-H_SLIDE_SPEED);
            else
                robot.linearSlide.motors[1].setPower(0.0);

            if(gamepad1.dpad_up)
                robot.hook.servos[0].setPosition(robot.hook.servos[0].getPosition() - HOOK_INCREMENT);
            else if(gamepad1.dpad_down)
                robot.hook.servos[0].setPosition(robot.hook.servos[0].getPosition() + HOOK_INCREMENT);

            if(gamepad1.guide)
                isStopped = true;

            if(isRobotCentric)
                robot.driveTrain.driveRobotCentric(
                        MathUtil.deadZone(gamepad1.left_stick_x, JOYSTICK_DEADZONE),
                        MathUtil.deadZone(-gamepad1.left_stick_y, JOYSTICK_DEADZONE),
                        MathUtil.deadZone(gamepad1.right_stick_x, JOYSTICK_DEADZONE),
                        isSmoothed, scaling);
            else
                robot.driveTrain.driveFieldCentric(
                        MathUtil.deadZone(gamepad1.left_stick_x, JOYSTICK_DEADZONE),
                        MathUtil.deadZone(-gamepad1.left_stick_y, JOYSTICK_DEADZONE),
                        MathUtil.deadZone(gamepad1.right_stick_x, JOYSTICK_DEADZONE),
                        isSmoothed, scaling);

            if(gamepad1.b)
                robot.linearSlide.servos[0].setPosition(robot.linearSlide.servos[0].getPosition() + GRIPPER_INCREMENT);
            else if(gamepad1.x)
                robot.linearSlide.servos[0].setPosition(robot.linearSlide.servos[0].getPosition() - GRIPPER_INCREMENT);

            telemetry.addData("isSmoothed", isStopped);
            telemetry.addData("isGripped", isGripped);
            telemetry.addData("scaling", scaling);
            robot.updateTelemetry();
            stickyGamepad1.update();
        }
    }
}
