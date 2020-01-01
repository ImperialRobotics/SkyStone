package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.StickyGamepad;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

public class Robot {
    public List<Subsystem> subsystems;
    public DriveTrain driveTrain;
//    public Intake intake;
    public LinearSlide linearSlide;

    private Intake.IntakeState intakeState;
    private OpMode opMode;

    //----------------------------------------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------------------------------------

    public Robot(LinearOpMode opMode, boolean autonomous) {
        this.opMode = opMode;

        //initialize subsystems
        driveTrain = new DriveTrain(opMode, autonomous);
//        intake = new Intake(opMode, autonomous);
        linearSlide = new LinearSlide(opMode, autonomous);
        subsystems = Arrays.asList(driveTrain, linearSlide);

        intakeState = Intake.IntakeState.INTAKE;
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry
    //----------------------------------------------------------------------------------------------

    public void updateTelemetry() {
        for(Subsystem subsystem: subsystems)
            for(Map.Entry<String, Object> entry: subsystem.updateTelemetry().entrySet())
                opMode.telemetry.addData(entry.getKey(), entry.getValue());
            opMode.telemetry.addLine();
            opMode.telemetry.update();
    }
}
