package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Mat;
import org.opencv.android.Utils;

public class VisionUtil {
    private static final String VUFORIA_KEY = "AQOGHXH/////AAABme0x8ObyAUzii3UV4T02I5lw0gxq8oJY69zk3Asw0BUT+3jrPNRQxi4JmOTUmEnIT4f536LplHYTEiTsB9RU4RE7KqhhyQcQWo9oJZLyFU7m1xoMB/dhbyTJ7i0RZcyqsK1QmYd3Ihu0XdJBG58YBrDctb5aJi+zG2tgezJo2zceT7sNMI15rk2uV+vl3C3RWxFwJOH9SiSIE5cKdOqUmOMGNjbULVhU8IeI+EWt+RqX9jmiHXRoe7o2iqODLTUWKb8Btn6O9tPQfxT4FwALD6Ss1wHTySzs5A3j0Y4t7K6NlbW+74UUI/gQmsTnnza/4fNMTTAaquanDHRYbuc6e+zvXAOMa9J6Y67jDSdpWv95";

    public static Mat bitmapToMat (Bitmap bit, int cvType) {
        Mat newMat = new Mat(bit.getHeight(), bit.getWidth(), cvType);
        Utils.bitmapToMat(bit, new Mat(bit.getHeight(), bit.getWidth(), cvType));

        return newMat;
    }

    public static VuforiaLocalizer getVuforiaLocalizer(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);
        return vuforia;
    }

    public enum SkystonePosition {
        LEFT, CENTER, RIGHT;
    }

}