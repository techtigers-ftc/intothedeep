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

    // Orientation exponential function parameters
    private static final double ORIENTATION_V_COMPRESS = 0.719428;
    private static final double ORIENTATION_BASE = 1.0629;
    private static final double ORIENTATION_TURN_THRESHOLD = 111;


    // Forward offset logistic function parameters
    private static final double FORWARD_FLOOR = 1.34;
    private static final double FORWARD_C = -1.77;
    private static final double FORWARD_H_STRETCH = 7390.34;
    private static final double FORWARD_RATE = 2.38;

    // Horizontal offset cube root function parameters
    private static final double LATERAL_V_STRETCH = 0.82;
    private static final double LATERAL_H_SHIFT = -2.7;
    private static final double LATERAL_V_SHIFT = 2.47;

    // Lateral bounds
    private double lateralLowerBound = -5;
    private double lateralUpperBound = 1;

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

    private double distanceBetweenPoints(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
    }

    private double getBlockWidth(LLResultTypes.DetectorResult detection) {
        // Width is the distance between the top right and bottom right corners of the detection
        return distanceBetweenPoints(detection.getTargetCorners().get(1).get(0), detection.getTargetCorners().get(1).get(1),
                detection.getTargetCorners().get(2).get(0), detection.getTargetCorners().get(2).get(1));
    }

    private double getClawAngle(LLResultTypes.DetectorResult detection) {
        double blockWidth = getBlockWidth(detection);
        RobotLog.dd(tag, "block width detection:%f", blockWidth);
        double distance = getBlockDistances(detection)[1];
//        double widthScalar = 0.000523061 * Math.pow(distance, 4)-0.0227837 * Math.pow(distance, 3)+0.348316 * Math.pow(distance, 2)-2.09751 * distance+5.15031;
        double widthScalar = 0.000447842 * Math.pow(distance, 4) - 0.020271 * Math.pow(distance, 3) + 0.320859 * Math.pow(distance, 2) - 1.98378 * distance + 5.00345;
        RobotLog.dd(tag, "width scalar: %f", widthScalar);
//        double widthScalar = ORIENTATION_V_COMPRESS * Math.pow(ORIENTATION_BASE, distance);
        double normalizedBlockWidth = blockWidth * widthScalar;
        RobotLog.dd(tag, "block width normalized:%f", normalizedBlockWidth);
        if (normalizedBlockWidth > ORIENTATION_TURN_THRESHOLD) {
            return IntakeSubsystem.CLAW_ROTATION_PICKUP_POSITION;
        } else {
            return 180;
        }
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
            robotState.setCoarseBlockDetectionState(state);
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
        // Gets the raw ty values from the limelight and converts them to rough forward distances
        double rawTy = detection.getTargetYDegrees();
        double ty = LIMELIGHT_DOWNWARD_ANGLE - rawTy;
        double yDist = LIMELIGHT_VERTICAL_HEIGHT * (1 / Math.tan(Math.toRadians(ty))) - LIMELIGHT_INTAKE_OFFSET;
        // Uses a logistic correction function to correct the y distance to the final y distance
        double YCorrectiveFactor = (FORWARD_C / (1 + FORWARD_H_STRETCH * Math.pow(Math.E, -FORWARD_RATE * yDist))) + FORWARD_FLOOR;
        RobotLog.dd(tag, "Forward Corrective Factor:%f", YCorrectiveFactor);
        double finalYDist = yDist - YCorrectiveFactor + 0.375;
        // Gets the raw tx values and converts them to rough lateral distances
        double rawTx = detection.getTargetXDegrees();
        double xDist = (finalYDist + LIMELIGHT_INTAKE_OFFSET) * Math.tan(Math.toRadians(rawTx)) - LIMELIGHT_X_OFFSET;
        double finalXDist = xDist + 0.4;

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
     * @return lower bound for determining if blocks are too far laterally
     */
    public double getLateralLowerBound() {
        return lateralLowerBound;
    }

    /**
     * Sets the lower bound for determining if blocks are too far laterally
     * @param lateralLowerBound the bound to set
     */
    public void setLateralLowerBound(double lateralLowerBound) {
        this.lateralLowerBound = lateralLowerBound;
    }

    /**
     * @return upper bound for determining if blocks are too far laterally
     */
    public double getLateralUpperBound() {
        return lateralUpperBound;
    }

    /**
     * Sets the upper bound for determining if blocks are too far laterally
     * @param lateralUpperBound the bound to set
     */
    public void setLateralUpperBound(double lateralUpperBound) {
        this.lateralUpperBound = lateralUpperBound;
    }

    /**
     * Goes through all of the limelight detections, checking to find the block that is deemed
     * most ideal to pickup; also sets the block detection state
     *
     * @param detections the list of neural detections to check
     */
    private void setBlockAttributes(List<LLResultTypes.DetectorResult> detections) {
        double distance = 1000;
        // List of valid detections
        ArrayList<LLResultTypes.DetectorResult> validDetections = new ArrayList<>();
        // List of detections within the distance ranges set
        ArrayList<LLResultTypes.DetectorResult> greatDetections = new ArrayList<>();
        // The detection closest to the robot
        LLResultTypes.DetectorResult bestDetection = null;

        // Checks to see if there are any detected blocks
        for (LLResultTypes.DetectorResult detection : detections) {
            // Checks to see that there is actually a detection
            // (The limelight returns 0 for tx and ty values if there is no detection)
            if (detection.getTargetXDegrees() != 0 && detection.getTargetYDegrees() != 0) {
                // Adds a block to the valid detections list if it is the preferred color
                if (isDetectionPreferredColor(detection)) {
                    validDetections.add(detection);
                }
            }
        }

        // Sets the block detection state to not detected if no blocks are detected
        if (validDetections.isEmpty()) {
            replaceCache(BlockDetectionState.NOT_DETECTED);
            return;
        }

        // Checks to see if the valid detections are  within the set distance ranges
        for (LLResultTypes.DetectorResult validDetection : validDetections) {
            double xDist = getBlockDistances(validDetection)[0];
            double yDist = getBlockDistances(validDetection)[1];
            if (xDist >= lateralLowerBound && xDist <= lateralUpperBound && yDist <= IntakeSubsystem.SLIDES_MAX - 0.25) {
                greatDetections.add(validDetection);
            }
        }

        // Sets the block detection state to too far if all the detected blocks aren't within the
        // set distance ranges
        if (greatDetections.isEmpty()) {
            replaceCache(BlockDetectionState.TOO_FAR);
            return;
        }

        // Finds the block closest to the robot from the list of valid detections within the set bounds
        for (LLResultTypes.DetectorResult greatDetection : greatDetections) {
            if (getWeightedEuclideanDistance(getBlockDistances(greatDetection)) < distance) {
                distance = getWeightedEuclideanDistance(getBlockDistances(greatDetection));
                bestDetection = greatDetection;
            }
        }

        // Sets the block detection state to detected
        robotState.setCoarseBlockDetectionState(BlockDetectionState.DETECTED);
        // Sets the coarse forward and lateral distances of the block found to be closest to the robot
        robotState.setBlockLateralCoarse(getBlockDistances(bestDetection)[0]);
        robotState.setBlockForwardCoarse(getBlockDistances(bestDetection)[1]);
//        robotState.setBlockOrientation(getClawAngle(bestDetection));
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
