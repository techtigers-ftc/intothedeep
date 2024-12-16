package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.GlobalConstants;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;

import java.util.List;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem which saves a detected sample's attributes into RobotState
 */
@Config
public class LimelightSubsystem extends CloseableSubsystem {
    private static final int NEURAL_DETECTOR_PIPELINE = 4;
    public static double TARGET_POINT_X = 450;
    public static double TARGET_POINT_Y = 360;
    private final RobotState robotState;
    private final Limelight3A limelight;
    private double height;
    private double xOffset;
    private double yOffset;
    private double downwardAngle;

    /**
     * Constructor for the LimelightSubsystem
     *
     * @param hardwareMap   Used to get the limelight camera from list of hardware devices
     * @param robotState    Used to set limelight values in robotstate
     * @param height        How high the limelight is off the ground
     * @param xOffset       Lateral distance of limelight from robot's center
     * @param yOffset       Distance from the limelight to the front of the slides
     * @param downwardAngle The angle the limelight is facing, in degrees
     */
    public LimelightSubsystem(HardwareMap hardwareMap, RobotState robotState, double height, double xOffset, double yOffset, double downwardAngle) {
        this.robotState = robotState;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.height = height;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.downwardAngle = downwardAngle;
    }

    @Override
    public void init() {
        limelight.setPollRateHz(50);
        limelight.start();
        limelight.pipelineSwitch(NEURAL_DETECTOR_PIPELINE);
    }

    /**
     * Finds the distance of a point from the center of the limelight
     *
     * @param x The x coordinate of the point to check
     * @param y The y coordinate of the point to check
     * @return The distance from the point to the center of the limelight
     */
    private double distanceFromExtensionPoint(double x, double y) {
        return Math.sqrt(Math.pow(x - TARGET_POINT_X, 2) + Math.pow(y - TARGET_POINT_Y, 2));
    }

    /**
     * Determines if a detection is the preferred color based on the robot's current block color preference
     *
     * @param detection The detection to check
     * @return True if the detection is the preferred color, false otherwise
     */
    private boolean isDetectionPreferredColor(LLResultTypes.DetectorResult detection) {
        // Gets the current block color preference
        BlockColorPreference colorPreference = robotState.getBlockColorPreference();
        // Gets the color of the alliance block, based on the global constants boolean
        String allianceBlock = GlobalConstants.getInstance().isRed ? "redsample" : "bluesample";
        // Gets the color of the detection
        String detectionColor = detection.getClassName();
        // If you detect a yellow sample
        if (detectionColor == "yellowsample") {
            // If the color preference is not alliance, return true, otherwise return false
            if (colorPreference != BlockColorPreference.ALLIANCE) {
                return true;
            } else {
                return false;
            }
            // If you detect an alliance block
        } else if (detectionColor == allianceBlock) {
            // If the color preference is not yellow, return true, otherwise return false
            if (colorPreference != BlockColorPreference.YELLOW) {
                return true;
            } else {
                return false;
            }
            // If you detect a block that is not yellow or the color of your alliance, return false
        } else {
            return false;
        }
    }

    /**
     * Gets the coordinates, in pixels, of the top left and bottom right corners of the
     * neural detection that is closest to the center of the limelight frame
     *
     * @param detections The list of neural detections to check
     * @return The top left and bottom right corners of the best neural detection
     */
    private double[] getNeuralDetectorCorners(List<LLResultTypes.DetectorResult> detections) {
        // Initializes a few variables to be used for comparison of the different detections
        double topLeftX = 0;
        double topLeftY = 0;
        double bottomRightX = 0;
        double bottomRightY = 0;
        for (LLResultTypes.DetectorResult detection : detections) {
            // Gets the values for the detection to check
            double newTopLeftX = detection.getTargetCorners().get(0).get(0);
            double newTopLeftY = detection.getTargetCorners().get(0).get(1);
            // Determines whether a detection is closer to the center of the limelight
            // than a detection that has already been made
            if ((distanceFromExtensionPoint(newTopLeftX, newTopLeftY) <
                    distanceFromExtensionPoint(topLeftX, topLeftY)) &&
                    isDetectionPreferredColor(detection)) {
                topLeftX = newTopLeftX;
                topLeftY = newTopLeftY;
                bottomRightX = detection.getTargetCorners().get(2).get(0);
                bottomRightY = detection.getTargetCorners().get(2).get(1);
            }
        }
        return new double[]{topLeftX, topLeftY, bottomRightX, bottomRightY};
    }

    private double[] getCenterCoordinates(LLResultTypes.DetectorResult detection) {
        double topLeftX = detection.getTargetCorners().get(0).get(0);
        double topLeftY = detection.getTargetCorners().get(0).get(1);
        double bottomRightX = detection.getTargetCorners().get(2).get(0);
        double bottomRightY = detection.getTargetCorners().get(2).get(1);
        return new double[]{(topLeftX + bottomRightX) / 2, (topLeftY + bottomRightY) / 2};
    }

    private double[] getNeuralDetectorTargetDegrees(List<LLResultTypes.DetectorResult> detections) {
        // Initializes a few variables to be used for comparison of the different detections
        double centerX = 0;
        double centerY = 0;
        double targetXDegrees = 0;
        double targetYDegrees = 0;
        for (LLResultTypes.DetectorResult detection : detections) {
            // Gets the values for the detection to check
            double newCenterX = getCenterCoordinates(detection)[0];
            double newCenterY = getCenterCoordinates(detection)[1];
            // Determines whether a detection is closer to the center of the limelight
            // than a detection that has already been made
            if ((distanceFromExtensionPoint(newCenterX, newCenterY) <
                    distanceFromExtensionPoint(centerX, centerY)) && isDetectionPreferredColor(detection)) {
                // Asserts this new block detection as the one closest to the camera center
                centerX = newCenterX;
                centerY = newCenterY;
                // Sets the values of this specific block detection to be used in the periodic
                targetXDegrees = detection.getTargetXDegrees();
                targetYDegrees = detection.getTargetYDegrees();
            }
        }
        return new double[]{targetXDegrees, targetYDegrees};
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            double[] targetDegrees = getNeuralDetectorTargetDegrees(result.getDetectorResults());
            double tx = targetDegrees[0];
            double ty = downwardAngle - targetDegrees[1];
            double yDist = height * (1 / Math.tan(Math.toRadians(ty)));
            double finalYDist = yDist - yOffset;
            double xDist = yDist * Math.tan(Math.toRadians(tx)) - xOffset;
            robotState.setBlockForwardCoarse(finalYDist);
            robotState.setBlockLateralCoarse(xDist);
            RobotLog.dd("x and y dist", "x dist:%f, y dist:%f", xDist, finalYDist);
        }
    }

    @Override
    public void close() {
        limelight.close();
    }
}
