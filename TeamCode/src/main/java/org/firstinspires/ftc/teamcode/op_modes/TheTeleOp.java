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
 *  y - increment state of intake (INTAKE, SPIT, DISABLED)
 *  x - loosen gripper
 *  b - tighten gripper
 *  dpad_up - retract hook
 *  dpad_down - lower hook
 *  right_bumper - extend vertical linear slide
 *  left_bumper - retract vertical linear slide
 *  right_trigger - extend horizontal linear slide
 *  left_trigger - retract horizontal linear slide
 *
 *  a - toggle robot centric
 *  guide - emergency stop
 */

@TeleOp(name="the teleop")
public class TheTeleOp extends LinearOpMode {

    private Robot robot;
    private boolean isRobotCentric;
    private StickyGamepad stickyGamepad1;
    private boolean isStopped;

    @Override
    public void runOpMode() {

        robot = new Robot(this, false, false);
        stickyGamepad1 = new StickyGamepad(gamepad1);
        isRobotCentric = true;
        isStopped = false;

        waitForStart();
        while(opModeIsActive() && !isStopped) {
            if(isRobotCentric)
                robot.driveTrain.driveRobotCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            else
                robot.driveTrain.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

//            if(stickyGamepad1.y)
//                robot.intake.incrementState();
            if(stickyGamepad1.a)
                isRobotCentric = !isRobotCentric;

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

            if(gamepad1.b)
                robot.linearSlide.servos[0].setPosition(robot.linearSlide.servos[0].getPosition() + GRIPPER_INCREMENT);
            else if(gamepad1.x)
                robot.linearSlide.servos[0].setPosition(robot.linearSlide.servos[0].getPosition() - GRIPPER_INCREMENT);

//            if(gamepad1.dpad_up)
//                robot.hook.servos[0].setPosition(robot.hook.servos[0].getPosition() - HOOK_INCREMENT);
//            else if(gamepad1.dpad_down)
//                robot.hook.servos[0].setPosition(robot.hook.servos[0].getPosition() + HOOK_INCREMENT);

            if(gamepad1.guide)
                isStopped = true;

//            robot.intake.handleState();
            robot.updateTelemetry();
            stickyGamepad1.update();
        }
    }
}
