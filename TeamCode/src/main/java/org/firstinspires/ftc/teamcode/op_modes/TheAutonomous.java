package org.firstinspires.ftc.teamcode.op_modes;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline;
import org.firstinspires.ftc.teamcode.vision.VisionUtil;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

@Autonomous(name="The Autonomous", group="LinearOpMode")
public class TheAutonomous extends LinearOpMode {

    private Robot robot;
    private VuforiaLocalizer vuforia;
    private VisionPipeline visionPipeline;
    private FtcDashboard dashboard;

    private int pos = 1;
    private VisionUtil.SkystonePosition skystonePosition;

    @Override
    public void runOpMode() {
        robot = new Robot(this, true, false);
        visionPipeline = new VisionPipeline();
        vuforia = VisionUtil.getVuforiaLocalizer(hardwareMap);
        dashboard = FtcDashboard.getInstance();

        while (!opModeIsActive() && !isStopRequested()) {
            VuforiaLocalizer.CloseableFrame vuFrame = null;
            if (!vuforia.getFrameQueue().isEmpty()) {
                try {
                    vuFrame = vuforia.getFrameQueue().take();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }

                if (vuFrame == null) continue;

                for (int i = 0; i < vuFrame.getNumImages(); i++) {
                    Image img = vuFrame.getImage(i);
                    if (img.getFormat() == PIXEL_FORMAT.RGB565) {
                        Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
                        bm.copyPixelsFromBuffer(img.getPixels());
                        Mat mat = VisionUtil.bitmapToMat(bm, CvType.CV_8UC3);
                        Mat ret = visionPipeline.processFrame(mat);
                        telemetry.addData("SKYSTONE POS: ", visionPipeline.getVumarkLeftBoundary());
                        Bitmap displayBitmap = Bitmap.createBitmap(ret.width(), ret.height(), Bitmap.Config.RGB_565);
                        Utils.matToBitmap(ret, displayBitmap);
                        dashboard.sendImage(displayBitmap);
                        int vu = visionPipeline.getVumarkLeftBoundary();
                        skystonePosition = visionPipeline.getSkystonePosition();
                        if (vu < 250) pos = 0;
                        else if (vu > 250 && vu < 620) pos = 1;
                        else pos = 2;
                    }
                }
            }
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("pos", pos);
            packet.put("skystone pos", skystonePosition);
            packet.put("left boundary", visionPipeline.getVumarkLeftBoundary());
            dashboard.sendTelemetryPacket(new TelemetryPacket());
            telemetry.addData("left boundary: ",visionPipeline.getVumarkLeftBoundary());
            telemetry.update();

            switch(skystonePosition) {
                case LEFT:
                    break;
                case CENTER:
                    break;
                case RIGHT:
                    break;
            }
        }
    }
}
