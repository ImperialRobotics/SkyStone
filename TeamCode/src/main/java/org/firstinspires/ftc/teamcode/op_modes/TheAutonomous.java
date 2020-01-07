package org.firstinspires.ftc.teamcode.op_modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name="the autonomous")
public class TheAutonomous extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true, false);

        telemetry.addData("Status", "waiting for start");
        waitForStart();
        robot.driveTrain.drive(24);
        robot.driveTrain.rotate(90);
        robot.driveTrain.strafe(24);

        robot.driveTrain.strafe(11);
        robot.driveTrain.drive(30);
//        robot.hook.deploy();
        robot.driveTrain.drive(-16);
        robot.driveTrain.rotate(90);
        robot.driveTrain.drive(-46);
    }
}
