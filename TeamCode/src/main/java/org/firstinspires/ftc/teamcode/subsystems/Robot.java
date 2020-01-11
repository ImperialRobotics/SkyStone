package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.StickyGamepad;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

public class Robot {
    public List<Subsystem> subsystems;
    public DriveTrain driveTrain;
    public LinearSlide linearSlide;
    public Hook hook;
    public FtcDashboard dashboard;

    private boolean usingDashboard;
    private OpMode opMode;

    //----------------------------------------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------------------------------------

    public Robot(LinearOpMode opMode, boolean autonomous, boolean usingDashboard) {
        this.opMode = opMode;

        //initialize subsystems
        driveTrain = new DriveTrain(opMode, autonomous);
        linearSlide = new LinearSlide(opMode, autonomous);
        hook = new Hook(opMode, autonomous);
        subsystems = Arrays.asList(driveTrain, linearSlide, hook);


        this.usingDashboard = usingDashboard;
        if(usingDashboard)
            this.dashboard = FtcDashboard.getInstance();
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry
    //----------------------------------------------------------------------------------------------

    public void updateTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        for(Subsystem subsystem: subsystems) {
            for (Map.Entry<String, Object> entry : subsystem.updateTelemetry().entrySet()) {
                opMode.telemetry.addData(entry.getKey(), entry.getValue());
                packet.put(entry.getKey(), entry.getValue());
            }
            opMode.telemetry.addLine();
            packet.addLine("");
        }
        opMode.telemetry.update();
        if(usingDashboard)
            dashboard.sendTelemetryPacket(packet);
    }
}
