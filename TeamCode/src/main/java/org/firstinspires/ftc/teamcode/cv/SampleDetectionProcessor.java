package org.firstinspires.ftc.teamcode.cv;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@Config
public class SampleDetectionProcessor implements VisionProcessor {
    public static double SMALLEST_BLOCK_ALLOWED_AREA = 30000;
    private Scalar UPPER_BOUND = new Scalar(50, 255, 255);
    private Scalar LOWER_BOUND = new Scalar(10, 10, 20);
    private double[] foundSample = new double[]{-1,-1,-1};
    private boolean isBlockDetected;
    public static int ERODE_SIZE = 5;
    private double detectCounter = 0;
    private final static double COUNTER_LIMIT = 0;

    private double camWidthPixels = 1280;
    private double camHeightPixels = 720;
    private double camDistToGround = 5;

    private double camWidthIn = 5.5;
    private double camHeightIn = 3;
    private double camLateralFOV = Math.toDegrees(Math.tan(camWidthIn/(2*camDistToGround)));
    private double camForwardFOV = Math.toDegrees(Math.tan(camHeightIn/(2*camDistToGround)));

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    private static List<MatOfPoint> getCanny(Mat frame) {
        RobotLog.dd("pipeline", "doing canny");
        Mat edges = new Mat();
        RobotLog.dd("pipeline", "eroding");
        Imgproc.erode(frame, frame, new Mat(ERODE_SIZE, ERODE_SIZE, Imgproc.MORPH_RECT));
        Imgproc.erode(frame, frame, new Mat(ERODE_SIZE, ERODE_SIZE, Imgproc.MORPH_RECT));
        Imgproc.erode(frame, frame, new Mat(ERODE_SIZE, ERODE_SIZE, Imgproc.MORPH_RECT));
        RobotLog.dd("pipeline", "doing gaussian blur");
//        Imgproc.GaussianBlur(frame, frame, new Size(0, 0), 0);
        RobotLog.dd("pipeline", "finished gaussian blur");

        Imgproc.Canny(frame, edges, 100, 200);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        return contours;
    }

    /**
     * @return Whether a block is detected
     */
    public boolean isBlockDetected() {
        return isBlockDetected;
    }

    /**
     * @return the x coordinates, y coordinates, and orientation of the found sample
     */
    public double[] getFoundSample() {
        return foundSample;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        RobotLog.dd("pipeline", "Processing frame");
        Mat processedMat = new Mat();

//        Mat edges = new Mat(frame.rows(), frame.cols(), frame.type());

        Imgproc.cvtColor(frame, processedMat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(processedMat, this.LOWER_BOUND, this.UPPER_BOUND, processedMat);

        List<MatOfPoint> contours = getCanny(processedMat);

        for (MatOfPoint contour : contours) {

            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

            Point[] rectPoints = new Point[4];
            rotatedRect.points(rectPoints);

            // Draw the rotated rectangle
            if (rotatedRect.size.width * rotatedRect.size.height > SMALLEST_BLOCK_ALLOWED_AREA) {
                detectCounter++;
                RobotLog.dd("pipeline", "counter: %f", detectCounter);
                if(detectCounter > COUNTER_LIMIT) {
                    double avgX = 0;
                    double avgY = 0;
                    for (int i = 0; i < 4; i++) {
                        //Draw the lines of the rectangles
                        Imgproc.line(frame, rectPoints[i], rectPoints[(i + 1) % 4], new Scalar(0, 255, 0), 2);
                        //Find the average points of the rectangle
                        avgX += rectPoints[i].x;
                        avgY += rectPoints[i].y;
                    }
                    //Find the average points of the rectangle
                    avgX /= 4;
                    avgY /= 4;
                    //Draw the center point of the rectangle
                    Imgproc.circle(frame, new Point(avgX, avgY), 20, new Scalar(255, 0, 0), -1);
                    //Next 2 lines for finding inches per pixel values
//                Imgproc.circle(frame, new Point(128,360), 20, new Scalar(255, 0, 0), -1);
//                Imgproc.circle(frame, new Point(768,360), 20, new Scalar(255, 0, 0), -1);

                    //Find one of the long sides of the rectangle
                    Point[] longSidePoints = Math.hypot(rectPoints[0].x - rectPoints[1].x, rectPoints[0].y - rectPoints[1].y)
                            > Math.hypot(rectPoints[1].x - rectPoints[2].x, rectPoints[1].y - rectPoints[2].y) ?
                            new Point[]{rectPoints[0], rectPoints[1]} :
                            new Point[]{rectPoints[1], rectPoints[2]};
                    //Find the slope of the long side
                    double slope = (longSidePoints[1].y - longSidePoints[0].y) / (longSidePoints[1].x - longSidePoints[0].x);
                    RobotLog.dd("Vision", String.valueOf(slope));
                    //Find out which sides are the short sides of the rectangle
                    Point[] shortSidePoints = Math.hypot(rectPoints[0].x - rectPoints[1].x, rectPoints[0].y - rectPoints[1].y)
                            < Math.hypot(rectPoints[1].x - rectPoints[2].x, rectPoints[1].y - rectPoints[2].y) ?
                            rectPoints :
                            new Point[]{rectPoints[1], rectPoints[2], rectPoints[3], rectPoints[0]};
                    //Find the midpoint of both short sides
                    Point shortMidpoint1 = new Point((shortSidePoints[0].x + shortSidePoints[1].x) / 2, (shortSidePoints[0].y + shortSidePoints[1].y) / 2);
                    Point shortMidpoint2 = new Point((shortSidePoints[2].x + shortSidePoints[3].x) / 2, (shortSidePoints[2].y + shortSidePoints[3].y) / 2);
                    //Draw center line
                    Imgproc.line(frame, shortMidpoint1, shortMidpoint2, new Scalar(0, 0, 255), 2);
                    //Adding information from the sample to eventually be added to robotState
                    double orientation = Math.toDegrees(Math.atan(slope));
                    foundSample = new double[]{avgX, avgY, orientation};
                    isBlockDetected = true;
                    break;
                }
            } else {
                detectCounter = 0;
                isBlockDetected = false;
            }

            // Approximate the contour to a polygon
//            MatOfPoint2f approxCurve = new MatOfPoint2f();
//            double epsilon = 0.04 * Imgproc.arcLength(contour2f, true);
//            Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);
//
//            // Convert back the polygon approximation to MatOfPoint
//            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            // Draw the polygon on the image
//            Imgproc.polylines(frame, List.of(points), true, new Scalar(0, 255, 0), 2)
        }
        return processedMat;
    }

    // TODO: Verify this math is correct (It is likely not)
    private double getRelativeBlockPosLateral(double pixelPos){
        return Math.toRadians(Math.atan((2*pixelPos*Math.tan(camLateralFOV))/camWidthPixels));
    }

    private double getRelativeBlockPosForward(double pixelPos){
        return Math.toRadians(Math.atan((2*pixelPos*Math.tan(camForwardFOV))/camHeightPixels));
    }



    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}