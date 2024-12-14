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
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@Config
public class SampleDetectionProcessor implements VisionProcessor {
    private Scalar UPPER_BOUND = new Scalar(170,255,255);
    private Scalar LOWER_BOUND = new Scalar(80,50,70);
    public static int ERODE_NUMBER = 10;
    private Sample foundSample;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    private static List<MatOfPoint> getCanny(Mat frame) {
        Mat edges = new Mat();
        Imgproc.erode(frame, frame,new Mat(ERODE_NUMBER,ERODE_NUMBER, Imgproc.MORPH_RECT));
        Imgproc.erode(frame, frame,new Mat(ERODE_NUMBER,ERODE_NUMBER, Imgproc.MORPH_RECT));
        Imgproc.erode(frame, frame,new Mat(ERODE_NUMBER,ERODE_NUMBER, Imgproc.MORPH_RECT));
        Imgproc.GaussianBlur(frame, frame, new Size(3, 3), 0);

        Imgproc.Canny(frame, edges, 100, 200);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        return contours;
    }

    public Sample getFoundSample() {
        return foundSample;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat processedMat = new Mat();

//        Mat edges = new Mat(frame.rows(), frame.cols(), frame.type());

        Imgproc.cvtColor(frame, processedMat, Imgproc.COLOR_RGB2HSV);
//
        Core.inRange(processedMat, this.LOWER_BOUND, this.UPPER_BOUND, processedMat);

        List<MatOfPoint> contours = getCanny(processedMat);

        for (MatOfPoint contour : contours) {

            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

            Point[] rectPoints = new Point[4];
            rotatedRect.points(rectPoints);

            // Draw the rotated rectangle
            if (rotatedRect.size.width * rotatedRect.size.height > 60000 && rotatedRect.size.width * rotatedRect.size.height < 145000) {
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
                Imgproc.circle(frame, new Point(avgX,avgY),20, new Scalar(255, 0, 0),-1);

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
                double orientation = Math.atan(slope);
                foundSample = new Sample(avgX, avgY, orientation, new Point[]{shortMidpoint1,shortMidpoint2});
                break;
            }

            // Approximate the contour to a polygon
//            MatOfPoint2f approxCurve = new MatOfPoint2f();
//            double epsilon = 0.04 * Imgproc.arcLength(contour2f, true);
//            Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);
//
//            // Convert back the polygon approximation to MatOfPoint
//            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            // Draw the polygon on the image
//            Imgproc.polylines(frame, List.of(points), true, new Scalar(0, 255, 0), 2);
        }
        return processedMat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
