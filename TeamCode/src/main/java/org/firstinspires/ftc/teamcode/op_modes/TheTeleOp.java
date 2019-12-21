package org.firstinspires.ftc.teamcode.op_modes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.StickyGamepad;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

/** Controls:
 * left_stick_x - horizontal movement
 * left_stick_y - vertical movement
 * right_stick_x - rotate
 *
 *  y - increment state of intake (INTAKE, SPIT, DISABLED)
 *  b - toggle robot centric
 */

public class TheTeleOp extends LinearOpMode {

    List<Subsystem> subsystems;
    DriveTrain driveTrain;
    Intake intake;
    StickyGamepad stickyGamepad1;
    boolean isRobotCentric;
    Intake.IntakeState intakeState;

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrain(this, false);
        intake = new Intake(this, false);
        subsystems = Arrays.asList(driveTrain, intake);

        stickyGamepad1 = new StickyGamepad(gamepad1);
        isRobotCentric = true;
        intakeState = Intake.IntakeState.INTAKE;

        waitForStart();
        while(opModeIsActive()) {
            if(isRobotCentric)
                driveTrain.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            else
                driveTrain.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if(stickyGamepad1.y)
                intake.incrementState();
            if(stickyGamepad1.b)
                isRobotCentric = !isRobotCentric;

            intake.handleState();
            handleTelemetry();
            stickyGamepad1.update();
        }
    }

    private void handleTelemetry() {
        for(Subsystem subsystem: subsystems)
            for(Map.Entry<String, Object> entry: subsystem.updateTelemetry().entrySet())
                telemetry.addData(entry.getKey(), entry.getValue());
        telemetry.addLine();
        telemetry.addData("drive orientation", isRobotCentric ? "robot centric" : "field centric");
        telemetry.update();
    }
}
