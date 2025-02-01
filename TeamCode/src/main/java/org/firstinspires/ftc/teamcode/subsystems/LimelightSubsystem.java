package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;

import java.util.List;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem which saves a detected sample's attributes into RobotState
 */
@Config
public class LimelightSubsystem extends CloseableSubsystem {
    private static final int BLOCK_CACHE_LIMIT = 3;
    private final static int NEURAL_DETECTOR_PIPELINE = 5;
    private final static double TARGET_POINT_X = 450;
    private final static double TARGET_POINT_Y = 360;
    private final static double LIMELIGHT_VERTICAL_HEIGHT = 10.5;
    private final static double LIMELIGHT_X_OFFSET = 4;
    private static final double LIMELIGHT_INTAKE_OFFSET = 8;
    private static final double LIMELIGHT_DOWNWARD_ANGLE = 25;
    private static final double SLIDES_INTAKE_OFFSET = 3;

    // Orientation logistic function parameters
    private static final double ORIENTATION_FLOOR = 0.00221939;
    private static final double ORIENTATION_C = 0.0725463;
    private static final double ORIENTATION_H_STRETCH = 4.40165;
    private static final double ORIENTATION_RATE = 0.0902167;

    // Forward offset logistic function parameters
    private static final double FORWARD_FLOOR = 1.34;
    private static final double FORWARD_C = -1.77;
    private static final double FORWARD_H_STRETCH = 7390.34;
    private static final double FORWARD_RATE = 2.38;

    // Horizontal offset cube root function parameters
    private static final double LATERAL_V_STRETCH = 0.82;
    private static final double LATERAL_H_SHIFT = -2.7;
    private static final double LATERAL_V_SHIFT = 2.47;

    // Block values
    private static final double BLOCK_WIDTH_VERTICAL = 1.5;
    private static final double BLOCK_WIDTH_HORIZONTAL = 3.5;
    private final RobotState robotState;
    private final Limelight3A limelight;

    private double framesCached;


    /**
     * Constructor for the LimelightSubsystem
     *
     * @param hardwareMap Used to get the limelight camera from list of hardware devices
     * @param robotState  Used to set limelight values in robotstate
     */
    public LimelightSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        this.robotState = robotState;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        framesCached = 1;
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
        String allianceBlock = robotState.isBlue() ? "bluesample" : "redsample";
        // Gets the color of the detection
        String detectionColor = detection.getClassName();
        // If you detect a yellow sample
        if (detectionColor.equals("yellowsample")) {
            // If the color preference is not alliance, return true, otherwise return false
            return colorPreference != BlockColorPreference.ALLIANCE;
            // If you detect an alliance block
        } else if (detectionColor.equals(allianceBlock)) {
            // If the color preference is not yellow, return true, otherwise return false
            return colorPreference != BlockColorPreference.YELLOW;
        }
        // If you detect a block that is not yellow or the color of your alliance, return false
        return false;
    }

    private double[] getCenterCoordinates(LLResultTypes.DetectorResult detection) {
        double topLeftX = detection.getTargetCorners().get(0).get(0);
        double topLeftY = detection.getTargetCorners().get(0).get(1);
        double bottomRightX = detection.getTargetCorners().get(2).get(0);
        double bottomRightY = detection.getTargetCorners().get(2).get(1);
        return new double[]{(topLeftX + bottomRightX) / 2, (topLeftY + bottomRightY) / 2};
    }

    private double distanceBetweenPoints(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
    }

    /**
     * Gets the target x and target y degrees of the neural detector detection
     * that is closest to the extension point of the limelight frame
     *
     * @param detections The list of detections of the neural detector
     * @return the target x and target y degrees of the best neural detection
     */
    private double[] getNeuralDetectorAttributes(List<LLResultTypes.DetectorResult> detections) {
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

    private void replaceCache(BlockDetectionState state) {
        boolean unCache = framesCached > BLOCK_CACHE_LIMIT
                || robotState.getRobotVelocity().getPoint().magnitude() > 1
                || robotState.getRobotVelocity().getHeading() > Math.toRadians(3);
        if (unCache) {
            robotState.setBlockDetectionState(state);
            framesCached = 1;
        } else {
            framesCached++;
        }
    }

    private double getBlockWidth(LLResultTypes.DetectorResult detection) {
        // Width is the distance between the top right and bottom right corners of the detection
        return distanceBetweenPoints(detection.getTargetCorners().get(1).get(0), detection.getTargetCorners().get(1).get(1),
                detection.getTargetCorners().get(2).get(0), detection.getTargetCorners().get(2).get(1));
    }

    private double getCorrectedYDist(double yDist) {
        double correctiveFactor = (FORWARD_C / (1 + FORWARD_H_STRETCH * Math.pow(Math.E, -FORWARD_RATE * yDist))) + FORWARD_FLOOR;
        RobotLog.dd(tag, "Forward Corrective Factor:%f", correctiveFactor);
        return yDist - correctiveFactor;
    }

    private double getCorrectedXDist(double xDist) {
        double correctiveFactor = LATERAL_V_STRETCH * Math.cbrt(xDist + LATERAL_H_SHIFT) + LATERAL_V_SHIFT;
        RobotLog.dd(tag, "Lateral Corrective Factor:%f", correctiveFactor);
        return xDist + correctiveFactor;
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            double[] angularValues = getNeuralDetectorAttributes(result.getDetectorResults());
            double tx = angularValues[0];
            double ty = LIMELIGHT_DOWNWARD_ANGLE - angularValues[1];
            double yDist = LIMELIGHT_VERTICAL_HEIGHT * (1 / Math.tan(Math.toRadians(ty))) - LIMELIGHT_INTAKE_OFFSET;
            double finalYDist = getCorrectedYDist(yDist);
            double xDist = (finalYDist + LIMELIGHT_INTAKE_OFFSET) * Math.tan(Math.toRadians(tx)) - LIMELIGHT_X_OFFSET;
            double finalXDist = getCorrectedXDist(xDist);
            if (xDist == -LIMELIGHT_X_OFFSET) {
                replaceCache(BlockDetectionState.NOT_DETECTED);
            } else if (finalYDist > IntakeSubsystem.SLIDES_MAX) {
                replaceCache(BlockDetectionState.TOO_FAR);
            } else {
                robotState.setBlockDetectionState(BlockDetectionState.DETECTED);
                robotState.setBlockForwardCoarse(finalYDist);
                robotState.setBlockLateralCoarse(finalXDist);
                robotState.setBlockOrientation(angularValues[2]);
            }
            RobotLog.dd(tag, "x dist:%f, y dist:%f", xDist, finalYDist);
        }
    }

    @Override
    public void close() {
        limelight.close();
    }
}
