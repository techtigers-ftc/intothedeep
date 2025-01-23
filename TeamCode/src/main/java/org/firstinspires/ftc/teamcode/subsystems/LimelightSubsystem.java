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
    private final static int NEURAL_DETECTOR_PIPELINE = 5;
    private final static double TARGET_POINT_X = 450;
    private final static double TARGET_POINT_Y = 360;
    private final RobotState robotState;
    private final Limelight3A limelight;
    private final static double LIMELIGHT_VERTICAL_HEIGHT = 10.5;
    private final static double LIMELIGHT_X_OFFSET = 2.9;
    private final double LIMELIGHT_Y_OFFSET = 6;
    private final double LIMELIGHT_DOWNWARD_ANGLE = 25;

    /**
     * Constructor for the LimelightSubsystem
     *
     * @param hardwareMap   Used to get the limelight camera from list of hardware devices
     * @param robotState    Used to set limelight values in robotstate
     */
    public LimelightSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        this.robotState = robotState;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
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
        if (targetXDegrees == 0 && targetYDegrees == 0) {
            robotState.setBlockDetectionState(BlockDetectionState.NOT_DETECTED);
        } else {
            robotState.setBlockDetectionState(BlockDetectionState.DETECTED);
        }
        return new double[]{targetXDegrees, targetYDegrees};
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            double[] angularValues = getNeuralDetectorAttributes(result.getDetectorResults());
            double tx = angularValues[0];
            double ty = LIMELIGHT_DOWNWARD_ANGLE - angularValues[1];
            double yDist = LIMELIGHT_VERTICAL_HEIGHT * (1 / Math.tan(Math.toRadians(ty)));
            double finalYDist = yDist - LIMELIGHT_Y_OFFSET;
            double xDist = yDist * Math.tan(Math.toRadians(tx)) - LIMELIGHT_X_OFFSET;
            if(xDist == -LIMELIGHT_X_OFFSET) {
                robotState.setBlockDetectionState(BlockDetectionState.NOT_DETECTED);
            } else if(finalYDist > IntakeSubsystem.SLIDES_MAX) {
                robotState.setBlockDetectionState(BlockDetectionState.TOO_FAR);
            } else {
                robotState.setBlockDetectionState(BlockDetectionState.DETECTED);
                robotState.setBlockForwardCoarse(finalYDist);
                robotState.setBlockLateralCoarse(xDist);
            }
            RobotLog.dd(tag, "x dist:%f, y dist:%f", xDist, finalYDist);
        }
    }

    @Override
    public void close() {
        limelight.close();
    }
}
