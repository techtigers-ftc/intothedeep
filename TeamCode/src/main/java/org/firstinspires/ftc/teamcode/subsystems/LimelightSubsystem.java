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

import java.util.ArrayList;
import java.util.List;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem which saves a detected sample's attributes into RobotState
 */
@Config
public class LimelightSubsystem extends CloseableSubsystem {
    private static final int BLOCK_CACHE_LIMIT = 3;
    private final static int NEURAL_DETECTOR_PIPELINE = 5;
    private final static double LIMELIGHT_VERTICAL_HEIGHT = 10.5;
    private final static double LIMELIGHT_X_OFFSET = 4;
    private static final double LIMELIGHT_INTAKE_OFFSET = 8;
    private static final double LIMELIGHT_DOWNWARD_ANGLE = 25;

    private static final double HEAVY_WEIGHT = 2;

    // Forward offset logistic function parameters
    private static final double FORWARD_FLOOR = 1.34;
    private static final double FORWARD_C = -1.77;
    private static final double FORWARD_H_STRETCH = 7390.34;
    private static final double FORWARD_RATE = 2.38;

    // Horizontal offset cube root function parameters
    private static final double LATERAL_V_STRETCH = 0.82;
    private static final double LATERAL_H_SHIFT = -2.7;
    private static final double LATERAL_V_SHIFT = 2.47;

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

    /**
     * Checks that the block is detected over multiple frames before setting the block detection state
     *
     * @param state the block detection state to set
     */
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

    /**
     * Gets the correct x and y distances of the block based on limelight tx and ty values
     *
     * @param detection the detection to find the values from
     * @return the corrected x and y distances of the block from the robot
     */
    private double[] getBlockDistances(LLResultTypes.DetectorResult detection) {
        double rawTy = detection.getTargetYDegrees();
        double ty = LIMELIGHT_DOWNWARD_ANGLE - rawTy;
        double yDist = LIMELIGHT_VERTICAL_HEIGHT * (1 / Math.tan(Math.toRadians(ty))) - LIMELIGHT_INTAKE_OFFSET;
        double YCorrectiveFactor = (FORWARD_C / (1 + FORWARD_H_STRETCH * Math.pow(Math.E, -FORWARD_RATE * yDist))) + FORWARD_FLOOR;
        RobotLog.dd(tag, "Forward Corrective Factor:%f", YCorrectiveFactor);
        double finalYDist = yDist - YCorrectiveFactor + 0.375;
        double rawTx = detection.getTargetXDegrees();
        double xDist = (finalYDist + LIMELIGHT_INTAKE_OFFSET) * Math.tan(Math.toRadians(rawTx)) - LIMELIGHT_X_OFFSET;
        double XCorrectiveFactor = LATERAL_V_STRETCH * Math.cbrt(xDist + LATERAL_H_SHIFT) + LATERAL_V_SHIFT;
        RobotLog.dd(tag, "Lateral Corrective Factor:%f", XCorrectiveFactor);
        double finalXDist = xDist + XCorrectiveFactor + 0.375;

        return new double[]{finalXDist, finalYDist};
    }

    /**
     * Gets the weighted euclidean distance of the block from the robot to favor blocks with vertical extension
     * rather than strafing to blocks
     *
     * @param distances the x and y distances of the block
     * @return the weighted euclidean distance of the block from the robot
     */
    private double getWeightedEuclideanDistance(double[] distances) {
        return Math.sqrt(Math.pow(distances[0], 2) + Math.pow(distances[1], 2) / HEAVY_WEIGHT);
    }

    /**
     * Goes through all of the limelight detections, checking to find the block that is deemed
     * most ideal to pickup; also sets the block detection state
     *
     * @param detections the list of neural detections to check
     */
    private void setBlockAttributes(List<LLResultTypes.DetectorResult> detections) {
        double distance = 1000;
        ArrayList<LLResultTypes.DetectorResult> validDetections = new ArrayList<>();
        ArrayList<LLResultTypes.DetectorResult> greatDetections = new ArrayList<>();
        LLResultTypes.DetectorResult bestDetection = null;

        for (LLResultTypes.DetectorResult detection : detections) {
            if (detection.getTargetXDegrees() != 0 && detection.getTargetYDegrees() != 0) {
                if (isDetectionPreferredColor(detection)) {
                    validDetections.add(detection);
                }
            }
        }

        if (validDetections.isEmpty()) {
            replaceCache(BlockDetectionState.NOT_DETECTED);
            return;
        }

        for (LLResultTypes.DetectorResult goodDetection : validDetections) {
            double xDist = getBlockDistances(goodDetection)[0];
            double yDist = getBlockDistances(goodDetection)[1];
            if (xDist >= -4 && xDist <= 3 && yDist <= IntakeSubsystem.SLIDES_MAX - 0.25) {
                greatDetections.add(goodDetection);
            }
        }

        if (greatDetections.isEmpty()) {
            replaceCache(BlockDetectionState.TOO_FAR);
            return;
        }

        for (LLResultTypes.DetectorResult greatDetection : greatDetections) {
            if (getWeightedEuclideanDistance(getBlockDistances(greatDetection)) < distance) {
                distance = getWeightedEuclideanDistance(getBlockDistances(greatDetection));
                bestDetection = greatDetection;
            }
            RobotLog.dd(tag, "Euclidian Distance: %f", getWeightedEuclideanDistance(getBlockDistances(greatDetection)));
        }

        RobotLog.dd(tag, "Best block euclidian distance: %f", getWeightedEuclideanDistance(getBlockDistances(bestDetection)));
        robotState.setBlockDetectionState(BlockDetectionState.DETECTED);
        robotState.setBlockLateralCoarse(getBlockDistances(bestDetection)[0]);
        robotState.setBlockForwardCoarse(getBlockDistances(bestDetection)[1]);
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            setBlockAttributes(result.getDetectorResults());
        }
    }

    @Override
    public void close() {
        limelight.close();
    }
}
