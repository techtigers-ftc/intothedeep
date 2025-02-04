package org.firstinspires.ftc.teamcode.cv;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.utils.RobotState;
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

/**
 * Processor for using the extension camera to detect blocks
 */
@Config
public class SampleDetectionProcessor implements VisionProcessor {
    public static final int WIDTH_RESOLUTION = 640;
    public static final int HEIGHT_RESOLUTION = 480;
    public static final int CAMERA_ANGLE_HEIGHT_CENTER = 280;
    private static final double CAMERA_VIEWING_VERTICAL_DIST = 5.25; // inches
    private static final double CAMERA_VIEWING_HORIZONTAL_DIST = 7.5; // inches
    private static final double CAMERA_VIEWING_VERTICAL_OVERLAP = 2.25; // inches
    private static final Translation2d center = new Translation2d(WIDTH_RESOLUTION / 2., CAMERA_ANGLE_HEIGHT_CENTER);
    private static final int ERODE_NUMBER = 10;

    //    Yellow color bounds:
    private final Scalar YELLOW_UPPER_BOUND = new Scalar(40, 255, 255);
    private final Scalar YELLOW_LOWER_BOUND = new Scalar(10, 50, 70);
    //    Red colors bounds:
    private final Scalar RED_UPPER_BOUND = new Scalar(10, 255, 255);
    private final Scalar RED_LOWER_BOUND = new Scalar(0, 50, 50);
    //    Blue color bounds:
    private final Scalar BLUE_UPPER_BOUND = new Scalar(120, 255, 255);
    private final Scalar BLUE_LOWER_BOUND = new Scalar(105, 110, 80);
    private final Mat processedMat = new Mat();
    private final RobotState robotState;


    // Vision Things
    private static final Mat edges = new Mat();
    private static final Mat erodeMat = new Mat(ERODE_NUMBER, ERODE_NUMBER, Imgproc.MORPH_RECT);
    private static final Size gaussianBlurSize = new Size(3, 3);
    private static final Mat hierarchy = new Mat();
    private static final Mat masked = new Mat();
    private static final Mat allianceMask = new Mat();
    private static final Mat yellowMask = new Mat();
    private static final Point[] rectPoints = new Point[4];
    private static final Scalar greenColor = new Scalar(0, 255, 0);
    private static final Scalar redColor = new Scalar(255, 0, 0);
    private static final Scalar blueColor = new Scalar(0, 0, 255);

    /**
     * Construct a SampleDetectionProcessor
     *
     * @param robotState The state of the robot
     */
    public SampleDetectionProcessor(RobotState robotState) {
        this.robotState = robotState;
    }

    private static List<MatOfPoint> getCanny(Mat frame) {
        Imgproc.erode(frame, frame, erodeMat);
        Imgproc.erode(frame, frame, erodeMat);
        Imgproc.erode(frame, frame, erodeMat);
        Imgproc.GaussianBlur(frame, frame, gaussianBlurSize, 0);

        Imgproc.Canny(frame, edges, 100, 200);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        return contours;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame, processedMat, Imgproc.COLOR_RGB2HSV);

        switch (robotState.getBlockColorPreference()) {
            case ALLIANCE:
                if (robotState.isBlue()) {
                    Core.inRange(processedMat, this.BLUE_LOWER_BOUND, this.BLUE_UPPER_BOUND, masked);
                } else {
                    Core.inRange(processedMat, this.RED_LOWER_BOUND, this.RED_UPPER_BOUND, masked);
                }
                break;
            case YELLOW:
                Core.inRange(processedMat, this.YELLOW_LOWER_BOUND, this.YELLOW_UPPER_BOUND, masked);
                break;
            case ANY:
                if (robotState.isBlue()) {
                    Core.inRange(processedMat, this.BLUE_LOWER_BOUND, this.BLUE_UPPER_BOUND, allianceMask);

                } else {
                    Core.inRange(processedMat, this.RED_LOWER_BOUND, this.RED_UPPER_BOUND, allianceMask);
                }
                Core.inRange(processedMat, this.YELLOW_LOWER_BOUND, this.YELLOW_UPPER_BOUND, yellowMask);
                Core.bitwise_or(allianceMask, yellowMask, masked);
        }

        List<MatOfPoint> contours = getCanny(masked);
        ArrayList<Pose2d> blockPoses = new ArrayList<>();

        for (MatOfPoint contour : contours) {

            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

            rotatedRect.points(rectPoints);

            // Draw the rotated rectangle
            if (rotatedRect.size.width * rotatedRect.size.height > 5000 && rotatedRect.size.width * rotatedRect.size.height < 145000) {
                double avgX = 0;
                double avgY = 0;
                for (int i = 0; i < 4; i++) {
                    //Draw the lines of the rectangles
                    Imgproc.line(frame, rectPoints[i], rectPoints[(i + 1) % 4], greenColor, 2);
                    //Find the average points of the rectangle
                    avgX += rectPoints[i].x;
                    avgY += rectPoints[i].y;
                }
                // Find the average points of the rectangle
                avgX /= 4;
                avgY /= 4;
                // Draw the center point of the rectangle
                Imgproc.circle(frame, new Point(avgX, avgY), 20, redColor, -1);

                // Find one of the long sides of the rectangle
                Point[] longSidePoints = Math.hypot(rectPoints[0].x - rectPoints[1].x, rectPoints[0].y - rectPoints[1].y)
                        > Math.hypot(rectPoints[1].x - rectPoints[2].x, rectPoints[1].y - rectPoints[2].y) ?
                        new Point[]{rectPoints[0], rectPoints[1]} :
                        new Point[]{rectPoints[1], rectPoints[2]};
                // Find the slope of the long side
                double slope = (longSidePoints[1].y - longSidePoints[0].y) / (longSidePoints[1].x - longSidePoints[0].x);

                // Find out which sides are the short sides of the rectangle
                Point[] shortSidePoints = Math.hypot(rectPoints[0].x - rectPoints[1].x, rectPoints[0].y - rectPoints[1].y)
                        < Math.hypot(rectPoints[1].x - rectPoints[2].x, rectPoints[1].y - rectPoints[2].y) ?
                        rectPoints :
                        new Point[]{rectPoints[1], rectPoints[2], rectPoints[3], rectPoints[0]};
                // Find the midpoint of both short sides
                Point shortMidpoint1 = new Point((shortSidePoints[0].x + shortSidePoints[1].x) / 2, (shortSidePoints[0].y + shortSidePoints[1].y) / 2);
                Point shortMidpoint2 = new Point((shortSidePoints[2].x + shortSidePoints[3].x) / 2, (shortSidePoints[2].y + shortSidePoints[3].y) / 2);
                //Draw center line
                Imgproc.line(frame, shortMidpoint1, shortMidpoint2, blueColor, 2);
                // Adding information from the sample to eventually be added to robotState
                double orientation = Math.atan(slope);
                blockPoses.add(new Pose2d(new Translation2d(avgX, avgY), new Rotation2d(orientation)));
                RobotLog.dd("Vision", avgX + " " + avgY + " " + orientation);
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                double epsilon = 0.04 * Imgproc.arcLength(contour2f, true);
                Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

                // Convert back the polygon approximation to MatOfPoint
                MatOfPoint points = new MatOfPoint(approxCurve.toArray());

                // Draw the polygon on the image
                Imgproc.polylines(frame, List.of(points), true, greenColor, 2);
            }
        }
        Pose2d bestBlock;
        try {
            bestBlock = blockPoses.get(0);
        } catch (Exception e) {
//            throw new RuntimeException(blockPoses.toString());
            return processedMat;
        }

        for (Pose2d pose : blockPoses) {
            if (pose.getTranslation().getDistance(center) > bestBlock.getTranslation().getDistance(center)) {
                bestBlock = pose;
            }
        }
        double avgY = bestBlock.getY();
        double avgX = bestBlock.getX();
        double orientation = bestBlock.getHeading();
//        robotState.setBlockForwardFine((HEIGHT_RESOLUTION - avgY) / HEIGHT_RESOLUTION * CAMERA_VIEWING_VERTICAL_DIST - CAMERA_VIEWING_VERTICAL_OVERLAP);
        robotState.setBlockForwardFine(avgY);
        robotState.setBlockLateralFine((avgX - WIDTH_RESOLUTION / 2.) / WIDTH_RESOLUTION * CAMERA_VIEWING_HORIZONTAL_DIST);
        robotState.setBlockOrientation((Math.toDegrees(orientation) + 180) % 180);

        edges.release();
        erodeMat.release();
        hierarchy.release();
        masked.release();
        allianceMask.release();
        yellowMask.release();
        return processedMat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }
}
