package org.firstinspires.ftc.teamcode.cv;

import android.graphics.Canvas;

import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A class for detecting sample orientation and fine distance
 */
public class BetterDetectionProcessor implements VisionProcessor {
    // Camera settings
    public static final int WIDTH_RESOLUTION = 640;
    public static final int HEIGHT_RESOLUTION = 480;
    public static final int CAMERA_ANGLE_HEIGHT_CENTER = 280;
    static final int CAMERA_FPS = 120;
    // Camera exposure settings
    static final double AUTO_EXPOSURE = 0.25;
    static final int EXPOSURE = -5;
    // Camera gain settings
    static final int GAIN = 0;
    // Wrapped OpenCV functions with tracking
    // Camera white balance settings
    static final int AUTO_WB = 0;
    static final double WB_RED = 1.0;
    static final double WB_GREEN = 1.0;
    static final double WB_BLUE = 1.0;
    // Edge detection parameters - initial values
    static final int CANNY_LOW = 120;
    static final int CANNY_HIGH = 200;
    static final int BLUR_SIZE = 13;
    static final int SOBEL_KERNEL = 7;
    // Color detection ranges for different color spaces
    static final Scalar HSV_BLUE_RANGE_LOW = new Scalar(90, 120, 40);
    static final Scalar HSV_BLUE_RANGE_HIGH = new Scalar(140, 255, 255);
    static final Scalar HSV_RED_RANGE_1_LOW = new Scalar(0, 120, 40);
    static final Scalar HSV_RED_RANGE_1_HIGH = new Scalar(10, 255, 255); // Red wraps around in HSV
    static final Scalar HSV_RED_RANGE_2_LOW = new Scalar(170, 120, 40);
    static final Scalar HSV_RED_RANGE_2_HIGH = new Scalar(180, 255, 255);
    static final Scalar HSV_YELLOW_RANGE_LOW = new Scalar(10, 120, 40);
    static final Scalar HSV_YELLOW_RANGE_HIGH = new Scalar(30, 255, 255);
    // Constants for filtering contours
    static final double SMALL_CONTOUR_AREA = 200;
    // Minimum average brightness threshold (0-255)
    static final double MIN_BRIGHTNESS_THRESHOLD = 60;
    private static final Translation2d center = new Translation2d(WIDTH_RESOLUTION / 2., CAMERA_ANGLE_HEIGHT_CENTER);
    // Track OpenCV function calls and timing
    // opencv_stats equivalent: a Map from function name to its count and total_time
    static Map<String, Stat> opencv_stats = new HashMap<>();
    private RobotState robotState;

    public BetterDetectionProcessor(RobotState robotState) {
        this.robotState = robotState;
    }

    // Helper function to update stats for tracked functions
    static void updateStat(String funcName, long nanoTime) {
        Stat stat = opencv_stats.get(funcName);
        if (stat == null) {
            stat = new Stat();
            opencv_stats.put(funcName, stat);
        }
        stat.count++;
        stat.total_time += nanoTime / 1e9; // convert nanoseconds to seconds
    }

    // cv2.split equivalent
    public static Mat[] split(Mat src) {
        long start = System.nanoTime();
        List<Mat> channels = new ArrayList<>();
        Core.split(src, channels);
        long end = System.nanoTime();
        updateStat("split", end - start);
        return channels.toArray(new Mat[0]);
    }

    // cv2.cvtColor equivalent
    public static Mat cvtColor(Mat src, int code) {
        long start = System.nanoTime();
        Mat dst = new Mat();
        Imgproc.cvtColor(src, dst, code);
        long end = System.nanoTime();
        updateStat("cvtColor", end - start);
        return dst;
    }

    // cv2.inRange equivalent
    public static Mat inRange(Mat src, Scalar lowerb, Scalar upperb) {
        long start = System.nanoTime();
        Mat dst = new Mat();
        Core.inRange(src, lowerb, upperb, dst);
        long end = System.nanoTime();
        updateStat("inRange", end - start);
        return dst;
    }

    // cv2.bitwise_and equivalent
    public static Mat bitwise_and(Mat src1, Mat src2, Mat mask) {
        long start = System.nanoTime();
        Mat dst = new Mat();
        Core.bitwise_and(src1, src2, dst, mask);
        long end = System.nanoTime();
        updateStat("bitwise_and", end - start);
        return dst;
    }

    public static Mat bitwise_and(Mat src1, Mat src2) {
        return bitwise_and(src1, src2, new Mat());
    }

    // cv2.bitwise_or equivalent
    public static Mat bitwise_or(Mat src1, Mat src2) {
        long start = System.nanoTime();
        Mat dst = new Mat();
        Core.bitwise_or(src1, src2, dst);
        long end = System.nanoTime();
        updateStat("bitwise_or", end - start);
        return dst;
    }

    // cv2.bitwise_not equivalent
    public static Mat bitwise_not(Mat src) {
        long start = System.nanoTime();
        Mat dst = new Mat();
        Core.bitwise_not(src, dst);
        long end = System.nanoTime();
        updateStat("bitwise_not", end - start);
        return dst;
    }

    // cv2.morphologyEx equivalent
    public static Mat morphologyEx(Mat src, int op, Mat kernel) {
        long start = System.nanoTime();
        Mat dst = new Mat();
        Imgproc.morphologyEx(src, dst, op, kernel);
        long end = System.nanoTime();
        updateStat("morphologyEx", end - start);
        return dst;
    }

    // cv2.GaussianBlur equivalent
    public static Mat GaussianBlur(Mat src, Size ksize, double sigmaX) {
        long start = System.nanoTime();
        Mat dst = new Mat();
        Imgproc.GaussianBlur(src, dst, ksize, sigmaX);
        long end = System.nanoTime();
        updateStat("GaussianBlur", end - start);
        return dst;
    }

    // cv2.Sobel equivalent
    public static Mat Sobel(Mat src, int ddepth, int dx, int dy, int ksize) {
        long start = System.nanoTime();
        Mat dst = new Mat();
        Imgproc.Sobel(src, dst, ddepth, dx, dy, ksize);
        long end = System.nanoTime();
        updateStat("Sobel", end - start);
        return dst;
    }

    public static ContoursResult findContoursCR(Mat src, int mode, int method) {
        long start = System.nanoTime();
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(src, contours, hierarchy, mode, method);
        long end = System.nanoTime();
        updateStat("findContours", end - start);
        return new ContoursResult(contours, hierarchy);
    }

    // cv2.drawContours equivalent
    public static void drawContours(Mat image, List<MatOfPoint> contours, int contourIdx, Scalar color, int thickness) {
        long start = System.nanoTime();
        Imgproc.drawContours(image, contours, contourIdx, color, thickness);
        long end = System.nanoTime();
        updateStat("drawContours", end - start);
    }

    public static void drawContours(Mat image, MatOfPoint contour, int contourIdx, Scalar color, int thickness) {
        List<MatOfPoint> contours = new ArrayList<>();
        contours.add(contour);
        drawContours(image, contours, contourIdx, color, thickness);
    }

    // cv2.dilate equivalent
    public static Mat dilate(Mat src, Mat kernel, int iterations) {
        long start = System.nanoTime();
        Mat dst = new Mat();
        Imgproc.dilate(src, dst, kernel, new Point(-1, -1), iterations);
        long end = System.nanoTime();
        updateStat("dilate", end - start);
        return dst;
    }

    // cv2.contourArea equivalent
    public static double contourArea(MatOfPoint contour) {
        long start = System.nanoTime();
        double area = Imgproc.contourArea(contour);
        long end = System.nanoTime();
        updateStat("contourArea", end - start);
        return area;
    }

    // cv2.fitEllipse equivalent (for a set of points, converted to MatOfPoint2f)
    public static RotatedRect fitEllipse(MatOfPoint2f points) {
        long start = System.nanoTime();
        RotatedRect rect = Imgproc.fitEllipse(points);
        long end = System.nanoTime();
        updateStat("fitEllipse", end - start);
        return rect;
    }

    // cv2.threshold equivalent
    public static Mat threshold(Mat src, double thresh, double maxVal, int type) {
        long start = System.nanoTime();
        Mat dst = new Mat();
        Imgproc.threshold(src, dst, thresh, maxVal, type);
        long end = System.nanoTime();
        updateStat("threshold", end - start);
        return dst;
    }

    // cv2.countNonZero equivalent
    public static int countNonZero(Mat src) {
        return Core.countNonZero(src);
    }

    // cv2.mean equivalent
    public static Scalar mean(Mat src, Mat mask) {
        return Core.mean(src, mask);
    }

    // cv2.boundingRect equivalent
    public static Rect boundingRect(MatOfPoint contour) {
        return Imgproc.boundingRect(contour);
    }

    // cv2.distanceTransform equivalent
    public static Mat distanceTransform(Mat src, int distanceType, int maskSize) {
        long start = System.nanoTime();
        Mat dst = new Mat();
        Imgproc.distanceTransform(src, dst, distanceType, maskSize);
        long end = System.nanoTime();
        updateStat("distanceTransform", end - start);
        return dst;
    }

    // Helper to clone a Mat (cv2.copy)
    public static Mat copy(Mat src) {
        return src.clone();
    }

    // calculate_angle: determines the angle from a contour using fitEllipse
    public static double calculate_angle(MatOfPoint contour) {
        if (contour.rows() < 5) {
            return 0;
        }
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
        RotatedRect ellipse = fitEllipse(contour2f);
        return ellipse.angle;
    }

    // draw_info: draws information onto the image
    public static void draw_info(Mat image, String color, double angle, Point center, int index, double area) {
        Imgproc.putText(image, "#" + index + ": " + color,
                new Point(center.x - 40, center.y - 60),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
        Imgproc.putText(image, "Angle: " + String.format("%.2f", angle),
                new Point(center.x - 40, center.y - 40),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
        Imgproc.putText(image, "Area: " + String.format("%.2f", area),
                new Point(center.x - 40, center.y - 20),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
        Imgproc.circle(image, center, 5, new Scalar(0, 255, 0), -1);
        double rad = Math.toRadians(90 - angle);
        Point lineEnd = new Point(center.x + 50 * Math.cos(rad), center.y - 50 * Math.sin(rad));
        Imgproc.line(image, center, lineEnd, new Scalar(0, 255, 0), 2);

    }

    // separate_touching_contours: separates touching contours based on area ratio
    public static List<MatOfPoint> separate_touching_contours(MatOfPoint contour, double min_area_ratio) {
        Rect rect = boundingRect(contour);
        // Create mask with dimensions (height, width)
        Mat mask = Mat.zeros(rect.height, rect.width, CvType.CV_8UC1);
        // Shift contour by subtracting (x, y)
        Point offset = new Point(rect.x, rect.y);
        Point[] pts = contour.toArray();
        Point[] shiftedPts = new Point[pts.length];
        for (int i = 0; i < pts.length; i++) {
            shiftedPts[i] = new Point(pts[i].x - offset.x, pts[i].y - offset.y);
        }
        MatOfPoint shifted_contour = new MatOfPoint(shiftedPts);
        Imgproc.drawContours(mask, Arrays.asList(shifted_contour), -1, new Scalar(255), -1);

        double original_area = Imgproc.contourArea(contour);
        List<MatOfPoint> max_contours = new ArrayList<>();
        int max_count = 1;

        Mat dist_transform = distanceTransform(mask, Imgproc.DIST_L2, 3);

        // Iterate thresholds from 0.1 to 0.9 in 9 steps
        for (int i = 0; i < 9; i++) {
            double thresholdVal = 0.1 + i * (0.8 / 8.0);
            Core.MinMaxLocResult mmr = Core.minMaxLoc(dist_transform);
            double maxVal = mmr.maxVal;
            Mat thresh = threshold(dist_transform, thresholdVal * maxVal, 255, Imgproc.THRESH_BINARY);
            thresh.convertTo(thresh, CvType.CV_8U);
            ContoursResult cr = findContoursCR(thresh, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            List<MatOfPoint> valid_contours = new ArrayList<>();
            for (MatOfPoint c : cr.contours) {
                if (Imgproc.contourArea(c) > original_area * min_area_ratio) {
                    valid_contours.add(c);
                }
            }
            if (valid_contours.size() > max_count) {
                max_count = valid_contours.size();
                max_contours = valid_contours;
            }
        }
        if (!max_contours.isEmpty()) {
            List<MatOfPoint> resultContours = new ArrayList<>();
            for (MatOfPoint c : max_contours) {
                Point[] cpts = c.toArray();
                for (int i = 0; i < cpts.length; i++) {
                    cpts[i] = new Point(cpts[i].x + offset.x, cpts[i].y + offset.y);
                }
                resultContours.add(new MatOfPoint(cpts));
            }
            return resultContours;
        }
        List<MatOfPoint> single = new ArrayList<>();
        single.add(contour);
        return single;
    }

    public static List<MatOfPoint> separate_touching_contours(MatOfPoint contour) {
        return separate_touching_contours(contour, 0.15);
    }

    // nothing: does nothing (used as a placeholder callback)
    public static void nothing(int x) {
        // pass
    }

    // Helper function to ensure a Mat is of type CV_32F (float)
    public static Mat convertToFloat(Mat src) {
        if (src.type() == CvType.CV_32F) {
            return src;
        }
        Mat dst = new Mat();
        src.convertTo(dst, CvType.CV_32F);
        return dst;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        try {
            int blur_size = BLUR_SIZE;
            int sobel_kernel = SOBEL_KERNEL;

            // Convert to HSV and denoise
            Mat hsv = cvtColor(frame, Imgproc.COLOR_BGR2HSV);

            // Split and display HSV channels
            Mat[] hsvChannels = split(hsv);
            Mat h = hsvChannels[0];
            Mat s = hsvChannels[1];
            Mat v = hsvChannels[2];

            Mat hsv_denoised = GaussianBlur(hsv, new Size(5, 5), 0);
            hsv_denoised = hsv;

            // Create masks for each color
            Mat blue_mask = inRange(hsv_denoised, HSV_BLUE_RANGE_LOW, HSV_BLUE_RANGE_HIGH);

            Mat red_mask1 = inRange(hsv_denoised, HSV_RED_RANGE_1_LOW, HSV_RED_RANGE_1_HIGH);
            Mat red_mask2 = inRange(hsv_denoised, HSV_RED_RANGE_2_LOW, HSV_RED_RANGE_2_HIGH);
            Mat red_mask = bitwise_or(red_mask1, red_mask2);

            Mat yellow_mask = inRange(hsv_denoised, HSV_YELLOW_RANGE_LOW, HSV_YELLOW_RANGE_HIGH);

            // Combine all color masks
            Mat combined_mask = bitwise_or(bitwise_or(blue_mask, red_mask), yellow_mask);

            Mat kernel = Mat.ones(5, 5, CvType.CV_8UC1);
            Mat masked_frame = bitwise_and(frame, frame, combined_mask);

            Mat gray_masked = cvtColor(masked_frame, Imgproc.COLOR_BGR2GRAY);

            // Edge detection pipeline
            Mat blurred = GaussianBlur(gray_masked, new Size(blur_size, blur_size), 0);

            Mat sobelx = Sobel(blurred, CvType.CV_32F, 1, 0, sobel_kernel);
            Mat sobely = Sobel(blurred, CvType.CV_32F, 0, 1, sobel_kernel);

            Mat magnitude = new Mat();
            Core.magnitude(convertToFloat(sobelx), convertToFloat(sobely), magnitude);
            Core.MinMaxLocResult mmlr = Core.minMaxLoc(magnitude);
            double maxMag = mmlr.maxVal;
            Mat magnitudeScaled = new Mat();
            Core.multiply(magnitude, new Scalar(255.0 / maxMag), magnitudeScaled);
            magnitudeScaled.convertTo(magnitudeScaled, CvType.CV_8U);

            // Threshold the magnitude image
            Mat edges = threshold(magnitudeScaled, 50, 255, Imgproc.THRESH_BINARY);

            edges = morphologyEx(edges, Imgproc.MORPH_CLOSE, kernel);

            edges = dilate(edges, Mat.ones(3, 3, CvType.CV_8UC1), 3);

            edges = bitwise_not(edges);
            edges = bitwise_and(edges, edges, combined_mask);

            ContoursResult cr = findContoursCR(edges, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            List<MatOfPoint> contours = cr.contours;

            Mat contour_frame = frame.clone();
            List<Map<String, Object>> gamePieces = new ArrayList<>();
            Mat gray = cvtColor(frame, Imgproc.COLOR_BGR2GRAY);

            for (int i = 0; i < contours.size(); i++) {
                MatOfPoint contour = contours.get(i);
                if (Imgproc.contourArea(contour) < SMALL_CONTOUR_AREA) {
                    continue;
                }
                List<MatOfPoint> sepContours = separate_touching_contours(contour);
                for (MatOfPoint sep_contour : sepContours) {
                    Mat mask = Mat.zeros(gray.size(), CvType.CV_8UC1);
                    Imgproc.drawContours(mask, Arrays.asList(sep_contour), -1, new Scalar(255), -1);

                    if (mean(gray, mask).val[0] < MIN_BRIGHTNESS_THRESHOLD) {
                        continue;
                    }

                    Moments M = Imgproc.moments(sep_contour);
                    if (M.m00 == 0) {
                        continue;
                    }

                    Point center = new Point(M.m10 / M.m00, M.m01 / M.m00);
                    double angle = calculate_angle(sep_contour);
                    double area = Imgproc.contourArea(sep_contour);

                    // Determine color of the piece by checking overlap with color masks
                    Mat roi_mask = Mat.zeros(gray.size(), CvType.CV_8UC1);
                    Imgproc.drawContours(roi_mask, Arrays.asList(sep_contour), -1, new Scalar(255), -1);

                    double total_area = (double) countNonZero(roi_mask);
                    double blue_overlap = (double) countNonZero(bitwise_and(blue_mask, roi_mask)) / total_area;
                    double red_overlap = (double) countNonZero(bitwise_and(red_mask, roi_mask)) / total_area;
                    double yellow_overlap = (double) countNonZero(bitwise_and(yellow_mask, roi_mask)) / total_area;

                    String color = "Unknown";
                    double max_overlap = Math.max(blue_overlap, Math.max(red_overlap, yellow_overlap));
                    if (max_overlap > 0.7) {
                        if (blue_overlap == max_overlap) {
                            color = "Blue";
                        } else if (red_overlap == max_overlap) {
                            color = "Red";
                        } else if (yellow_overlap == max_overlap) {
                            color = "Yellow";
                        }
                    }

                    Map<String, Scalar> color_map = new HashMap<>();
                    color_map.put("Blue", new Scalar(255, 0, 0));
                    color_map.put("Red", new Scalar(0, 0, 255));
                    color_map.put("Yellow", new Scalar(0, 255, 255));
                    if (color_map.containsKey(color)) {  // Only draw if a valid color was detected
                        Imgproc.drawContours(contour_frame, Arrays.asList(sep_contour), 0, color_map.get(color), 2);
                        draw_info(contour_frame, color, angle, center, gamePieces.size() + 1, area);

                        Map<String, Object> piece = new HashMap<>();
                        piece.put("index", gamePieces.size() + 1);
                        piece.put("color", color);
                        piece.put("position", center);
                        piece.put("angle", angle);
                        piece.put("area", area);
                        piece.put("brightness", mean(gray, mask).val[0]);
                        if (cr.hierarchy.rows() > 0 && cr.hierarchy.cols() > i && cr.hierarchy.get(0, i)[3] == -1) {
                            piece.put("hierarchy_level", "external");
                        } else {
                            piece.put("hierarchy_level", "internal");
                        }
                        gamePieces.add(piece);
                    }
                }
            }

            if (!contours.isEmpty()) {
                Map<String, Object> bestPiece = gamePieces.get(0);
                for (Map<String, Object> piece : gamePieces) {
                    Point piecePosition = (Point) piece.get("position");
                    Point bestPiecePosition = (Point) bestPiece.get("position");
                    double pieceDistance = Math.hypot(piecePosition.x - center.getX(), piecePosition.y - center.getY());
                    double bestPieceDistance = Math.hypot(bestPiecePosition.x - center.getX(), bestPiecePosition.y - center.getY());
                    if (pieceDistance < bestPieceDistance) {
                        bestPiece = piece;
                    }
                }
                robotState.setFineBlockDetectionState(BlockDetectionState.DETECTED);
                robotState.setBlockOrientation((double) bestPiece.get("angle"));
                robotState.setBlockForwardFine(((Point) bestPiece.get("position")).y);

                robotState.setBlockLateralFine(((Point) bestPiece.get("position")).x);

            } else {
                robotState.setFineBlockDetectionState(BlockDetectionState.NOT_DETECTED);
                robotState.setBlockForwardFine(-1);
                robotState.setBlockLateralFine(-1);
            }


            return contour_frame;
        } catch (Exception e) {
            System.out.println(frame + " Error: " + e.getMessage());
            return frame;
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    static class Stat {
        public int count;
        public double total_time;

        public Stat() {
            count = 0;
            total_time = 0;
        }
    }

    // processFrame: processes each frame from the camera

    // cv2.findContours equivalent - we create a container to hold contours and hierarchy
    public static class ContoursResult {
        public List<MatOfPoint> contours;
        public Mat hierarchy;

        public ContoursResult(List<MatOfPoint> contours, Mat hierarchy) {
            this.contours = contours;
            this.hierarchy = hierarchy;
        }
    }

}