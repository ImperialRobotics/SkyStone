package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.util.Constants.*;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.*;

public class VisionPipeline extends OpenCvPipeline{
    private final Point anchor = new Point(-1,-1);

    private int minX, minY = Integer.MAX_VALUE;
    private int maxX, maxY = -1 * Integer.MAX_VALUE;

    private int vumarkLeftBoundary = -1;

    private Mat mask = new Mat(), kernel = new Mat(), hierarchy = new Mat();

    private int inputWidth;

    private final int INDEX_ERROR = -2; // index error code

    public Mat processFrame(Mat input) {
        Mat workingMat = input.clone();
        Imgproc.cvtColor(workingMat,workingMat,Imgproc.COLOR_RGB2HSV); // convert to HSV space

        Core.inRange(workingMat, MIN_HSV, MAX_HSV, mask); // apply yellow filter
        Imgproc.erode(mask, mask, kernel, anchor, ERODE_ITERATIONS); // basically a faster blur
        Imgproc.dilate(mask, mask, kernel, anchor, DILATE_ITERATIONS); // remove noise

        List<MatOfPoint> stoneContours = new ArrayList<>();
        Imgproc.findContours(mask, stoneContours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE); // find block contours

        // remove noisy contours
        List<MatOfPoint> outputContours = new ArrayList<>();
        filterContours(stoneContours,outputContours, MIN_CONTOUR_AREA, MIN_CONTOUR_PERIMETER, MIN_CONTOUR_WIDTH, MIN_CONTOUR_HEIGHT);

        workingMat.release();

        // draw stones (but not skystones)
        if (minX > 1e5 || maxX < 0 || minY > 1e5 || maxY < 0) return input;

        Rect r = new Rect(new Point(minX, minY + 150), new Point(maxX,  maxY));
        Imgproc.rectangle(input, r, new Scalar(0,0,255));

        // crop verticallyx
        Mat cbMat = crop(input.clone(),new Point(0, minY + 150), new Point(input.width() - 1,  maxY));

        Imgproc.cvtColor(cbMat,cbMat,Imgproc.COLOR_RGB2YCrCb); // convert to ycrcb
        Core.extractChannel(cbMat, cbMat, 2); // extract cb channel

        Imgproc.threshold(cbMat, cbMat, CB_MIN, CB_MAX, Imgproc.THRESH_BINARY_INV); // binary mask to find the vumark

        List<MatOfPoint> stoneContours2 = new ArrayList<>();
        Imgproc.findContours(cbMat, stoneContours2, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE); // draw contours around the stones
        List<MatOfPoint> finalOutputContours = new ArrayList<>();
        resetRectangle();

        // build the max bounding rect
        filterContours(stoneContours2,finalOutputContours, MIN_CONTOUR_AREA, MIN_CONTOUR_PERIMETER, MIN_CONTOUR_WIDTH, MIN_CONTOUR_HEIGHT);
        if (finalOutputContours.size() == 0) return input;

        // crop so we only see the stones
        Rect allStonesRect = getMaxRectangle();
        Mat cbCrop = crop(cbMat, allStonesRect);

        vumarkLeftBoundary = getMaxDropoff(cbCrop);

        // this just estimates where the marker is, dont rely on this
        Imgproc.circle(input,new Point(vumarkLeftBoundary + 200, input.height() / 2), 60, new Scalar(255,0,0), 10);
        resetRectangle();

        inputWidth = input.width();

        return input;
    }

    private void filterContours(List<MatOfPoint> contours, List<MatOfPoint> outputContours, double minContourArea, double minContourPerimeter, double minContourWidth,
                                double minContourHeight) {
        for (MatOfPoint contour : contours) {
            Rect rect = Imgproc.boundingRect(contour);
            int x = rect.x;
            int y = rect.y;
            int w = rect.width;
            int h = rect.height;

            if (w < minContourWidth)
                continue;
            if (rect.area() < minContourArea)
                continue;
            if ((2 * w + 2 * h) < minContourPerimeter)
                continue;
            if (h < minContourHeight)
                continue;
            outputContours.add(contour);

            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (x + w > maxX) maxX = x + w;
            if (y + h> maxY) maxY = y + h;
        }

    }

    private Mat crop(Mat image, Point topLeftCorner, Point bottomRightCorner) {
        Rect cropRect = new Rect(topLeftCorner, bottomRightCorner);
        return new Mat(image, cropRect);
    }

    private Mat crop(Mat image, Rect rect) {
        return new Mat(image, rect);
    }

    private int getMaxDropoff(Mat image) {

        Mat columns = new Mat();
        Core.reduce(image, columns, 0, Core.REDUCE_SUM, 4);

        columns.convertTo(columns, CvType.CV_32S);

        int[] colsumArray = new int[(int)(columns.total()*columns.channels())];
        columns.get(0,0,colsumArray);
        for (int i = 0; i < colsumArray.length; i++) {
            colsumArray[i] /= 140;
        }


        for (int i = 0; i < colsumArray.length; i++) {
            if (colsumArray[i] < MAX_VUMARK_VALUE) {
                if (i + VALLEY_LENGTH > colsumArray.length - 1) return INDEX_ERROR;
                int[] slice = Arrays.copyOfRange(colsumArray, i, i+VALLEY_LENGTH);
                if (isLargeValley(slice, MAX_VUMARK_VALUE, 25)) return i;
            }
        }
        return -1;
    }

    private boolean isLargeValley(int[] slice, double maxValue, double thresh) {
        List<Integer> differences = new ArrayList<>(slice.length - 1);
        for (int i : slice) {
            if (i > maxValue) return false;

        }
        for (int i = 0; i < slice.length - 1; i++)
            differences.add(Math.abs(slice[i+1] - slice[i]));

        return Collections.max(differences) < thresh;
    }

    public int getVumarkLeftBoundary() { return vumarkLeftBoundary; }

    public VisionUtil.SkystonePosition getSkystonePosition() {
        if (vumarkLeftBoundary < (2 * inputWidth / 7))
            return VisionUtil.SkystonePosition.LEFT;
        else if (vumarkLeftBoundary > ( 5 * inputWidth / 7))
            return VisionUtil.SkystonePosition.CENTER;
        else
            return VisionUtil.SkystonePosition.RIGHT;
    }

    private void resetRectangle() {
        maxX = maxY = -1 * (int) 1e8;
        minX = minY = (int) 1e8;
    }

    private Rect getMaxRectangle() { return new Rect(new Point(minX, minY), new Point(maxX,maxY)); }

}