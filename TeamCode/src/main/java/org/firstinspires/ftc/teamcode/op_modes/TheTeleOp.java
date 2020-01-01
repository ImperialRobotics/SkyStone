package org.firstinspires.ftc.teamcode.op_modes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.StickyGamepad;

import static org.firstinspires.ftc.teamcode.util.Constants.*;


/** Controls:
 * left_stick_x - horizontal movement
 * left_stick_y - vertical movement
 * right_stick_x - rotate
 *
 *  y - increment state of intake (INTAKE, SPIT, DISABLED)
 *  right_bumper - extend vertical linear slide
 *  left_bumper - retract vertical linear slide
 *  right_trigger - extend horizontal linear slide
 *  left_trigger - retract horizontal linear slide
 *
 *  b - toggle robot centric
 */

public class TheTeleOp extends LinearOpMode {

    private Robot robot;
    private boolean isRobotCentric;
    private StickyGamepad stickyGamepad1;
    private boolean isStopped;

    @Override
    public void runOpMode() {

        robot = new Robot(this, false);
        stickyGamepad1 = new StickyGamepad(gamepad1);
        isRobotCentric = true;
        isStopped = false;

        waitForStart();
        while(opModeIsActive() && !isStopped) {
            if(isRobotCentric)
                robot.driveTrain.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            else
                robot.driveTrain.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

//            if(stickyGamepad1.y)
//                robot.intake.incrementState();
            if(stickyGamepad1.b)
                isRobotCentric = !isRobotCentric;

            if(stickyGamepad1.right_bumper)
                robot.linearSlide.motors[0].setPower(V_SLIDE_SPEED);
            else if(stickyGamepad1.left_bumper)
                robot.linearSlide.motors[0].setPower(-V_SLIDE_SPEED);
            else
                robot.linearSlide.motors[0].setPower(0);

            if(stickyGamepad1.right_trigger)
                robot.linearSlide.hSlide.setPosition(robot.linearSlide.hSlide.getPosition() + H_SLIDE_INCREMENT);
            else if(stickyGamepad1.left_trigger)
                robot.linearSlide.hSlide.setPosition(robot.linearSlide.hSlide.getPosition() - H_SLIDE_INCREMENT);

            if(gamepad1.guide)
                isStopped = true;

//            robot.intake.handleState();
            robot.updateTelemetry();
            stickyGamepad1.update();
        }
    }
}
