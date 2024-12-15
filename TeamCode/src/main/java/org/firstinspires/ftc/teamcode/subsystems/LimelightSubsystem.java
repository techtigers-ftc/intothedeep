package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;

import java.util.List;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem which saves a detected sample's attributes into RobotState
 */
@Config
public class LimelightSubsystem extends CloseableSubsystem {
    private static final int PYTHON_PIPELINE = 0;
    private static final int NEURAL_DETECTOR_PIPELINE = 4;
    public static double TARGET_POINT_X = 450;
    public static double TARGET_POINT_Y = 360;
    public static double FREEZING_CHECK = 900;
    private final RobotState robotState;
    private final Limelight3A limelight;
    private final ElapsedTime pipelineSwitchTimer;
    private double limelightPythonUpdating;
    private double limelightPythonFreezing;
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
        pipelineSwitchTimer = new ElapsedTime();
        limelightPythonUpdating = -1;
        limelightPythonFreezing = -1;
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
        pipelineSwitchTimer.reset();
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
     * Gets the coordinates, in pixels, of the top left and bottom right corners of the
     * neural detection that is closest to the center of the limelight frame
     *
     * @param detections The list of neural detections to check
     * @return The top left and bottom right corners of the best neural detection
     */
    private double[] getNeuralDetectorCorners(List<LLResultTypes.DetectorResult> detections, String sampleType) {
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
                    (detection.getClassName().equals(sampleType))) {
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

    private double[] getNeuralDetectorTargetDegrees(List<LLResultTypes.DetectorResult> detections, String sampleType) {
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
                    distanceFromExtensionPoint(centerX, centerY)) &&
                    (detection.getClassName().equals(sampleType))) {
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

    /**
     * Checks if the result of the python pipeline are valid and proceeds
     * to set them in robot state if they are valid
     *
     * @param pythonOutput The array of python pipeline outputs to check
     */
    private void setPythonOutput(double[] pythonOutput) {
        // Checks if the python outputs are valid
        if (pythonOutput[5] == 0) {
            robotState.setBlockDetectionState(BlockDetectionState.NOT_DETECTED);
            setPipelineAfterTime(100, NEURAL_DETECTOR_PIPELINE);
        } else if (pythonOutput[5] == 1) {
            robotState.setBlockDetectionState(BlockDetectionState.DETECTED);
        } else if (pythonOutput[5] == 2) {
            robotState.setBlockDetectionState(BlockDetectionState.TRACKING);
        }
    }

    /**
     * Switches the limelight to a pipeline after the pipelineSwitchTimer reaches past a certain time
     *
     * @param time          The time that the timer must pass
     * @param pipelineIndex The index of pipeline to switch to
     */
    private void setPipelineAfterTime(double time, int pipelineIndex) {
        if (pipelineSwitchTimer.milliseconds() > time) {
            limelight.pipelineSwitch(pipelineIndex);
            pipelineSwitchTimer.reset();
        }
    }

    /**
     * Checks if the python pipeline is updating and resets the pipeline if it is freezing
     *
     * @param pythonOutput The array of python pipeline outputs to check
     */
    private void pythonPipelineFreezeCheck(double[] pythonOutput) {
        // Checks to see if the updating value from the python pipeline has changed
        // since the last time it was checked
        if (pythonOutput[5] == limelightPythonUpdating) {
            // Checks if the python pipeline has been freezing for a certain amount of time
            if (limelightPythonFreezing >= FREEZING_CHECK) {
                // Switches the pipeline to the neural detector if it is freezing
                limelight.pipelineSwitch(NEURAL_DETECTOR_PIPELINE);
                pipelineSwitchTimer.reset();
                limelightPythonFreezing = 0;
                RobotLog.dd("TT-LLS", "resetting");
            } else {
                // Increments the freezing counter if the python pipeline is freezing
                limelightPythonFreezing++;
            }
        } else {
            limelightPythonFreezing = 0;
        }
        // Sets the updating value to the current value to check in the next cycle
        limelightPythonUpdating = pythonOutput[5];
    }

    @Override
    public void periodic() {

//        //NEW CODE
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            double[] targetDegrees = getNeuralDetectorTargetDegrees(result.getDetectorResults(), "yellowsample");
            double tx = targetDegrees[0];
            double ty = downwardAngle - targetDegrees[1];
//            double tx = result.getTx();
//            double ty = result.getTy();
            double yDist = height * (1 / Math.tan(Math.toRadians(ty)));
            double finalYDist = yDist - yOffset;
            double xDist = yDist * Math.tan(Math.toRadians(tx)) - xOffset;
            robotState.setBlockForwardCoarse(finalYDist);
            robotState.setBlockLateralCoarse(xDist);
            RobotLog.dd("x and y dist", "x dist:%f, y dist:%f", xDist, finalYDist);
        }

//        //OLD CODE
//        // Checks if the result is null
//        if (result != null) {
//            RobotLog.dd("TT-LLS", String.valueOf(result.getPipelineIndex()));
//            // Checks if the active pipeline is the python pipeline
//            if (result.getPipelineIndex() == PYTHON_PIPELINE) {
//                // Checks if the pickup is complete
//                // Gets the python outputs if the pickup is not complete
////                double[] pythonOutput = result.getPythonOutput();
//                // Checks if the python outputs are updating and resets the
//                // pipeline in order to unfreeze if needed
////                pythonPipelineFreezeCheck(pythonOutput);
//                // Checks the validity of the python outputs and sets them if valid,
//                // returns to neural detector if there are no results found
////                RobotLog.dd("python output", Arrays.toString(pythonOutput));
//                double tx = result.getTx();
//                double ty = downwardAngle - result.getTy();
//                double yDist = height * (1 / Math.tan(Math.toRadians(ty)));
//                double xDist = yDist * Math.tan(Math.toRadians(tx)) - xOffset;
//                yDist -= yOffset;
//                robotState.setBlockForwardCoarse(yDist);
//                robotState.setBlockLateralCoarse(xDist);
//                RobotLog.dd("tx", "tx:%f, ty:%f", tx, ty);
//                RobotLog.dd("x and y dist", "x dist:%f, y dist:%f", xDist, yDist);
//            } else if (result.getPipelineIndex() == NEURAL_DETECTOR_PIPELINE) {
//                double[] detectorCorners = getNeuralDetectorCorners(result.getDetectorResults(), "yellowsample");
//                limelight.updatePythonInputs(detectorCorners[0], detectorCorners[1],
//                        detectorCorners[2], detectorCorners[3], 0, 0, 0, 0);
//                RobotLog.dd("TT-LLS", Arrays.toString(
//                        new double[]{detectorCorners[0], detectorCorners[1], detectorCorners[2],
//                                detectorCorners[3]}));
//                // Switches to the python pipeline and begins tracking
//                setPipelineAfterTime(100, PYTHON_PIPELINE);
//            } else {
//                setPipelineAfterTime(100, NEURAL_DETECTOR_PIPELINE);
//            }
//        }
    }

    @Override
    public void close() {
        limelight.close();
    }
}
