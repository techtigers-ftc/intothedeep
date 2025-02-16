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
    private final static double LIMELIGHT_VERTICAL_HEIGHT = 10.5;
    private final static double LIMELIGHT_X_OFFSET = 4;
    private static final double LIMELIGHT_INTAKE_OFFSET = 8;
    private static final double LIMELIGHT_DOWNWARD_ANGLE = 25;

    private static final double HEAVY_WEIGHT = 2;

    private static final double WIDTH_RANGE = 6; // TODO: Tune properly
    private static final double HEIGHT_RANGE = 5.6;
    private static final double PIXELS_PER_INCH = 94.5;

    private final RobotState robotState;
    private final Limelight3A limelight;
    // Lateral bounds
    private double lateralLowerBound = -5;
    private double lateralUpperBound = 1;

    /**
     * Constructor for the LimelightSubsystem
     *
     * @param hardwareMap Used to get the limelight camera from list of hardware devices
     * @param robotState  Used to set limelight values in robotstate
     */
    public LimelightSubsystem(HardwareMap hardwareMap, RobotState robotState) {
        this.robotState = robotState;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    @Override
    public void init() {
        limelight.setPollRateHz(50);
        limelight.start();
        limelight.pipelineSwitch(3);
    }

    private double distanceBetweenPoints(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
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
     * Sets the lower bound for determining if blocks are too far laterally
     *
     * @param lateralLowerBound the bound to set
     */
    public void setLateralLowerBound(double lateralLowerBound) {
        this.lateralLowerBound = lateralLowerBound;
    }

    /**
     * Sets the upper bound for determining if blocks are too far laterally
     *
     * @param lateralUpperBound the bound to set
     */
    public void setLateralUpperBound(double lateralUpperBound) {
        this.lateralUpperBound = lateralUpperBound;
    }

    @Override
    public void periodic() {
        double blue = robotState.isBlue() ? 1 : 0;
        double red = robotState.isBlue() ? 0 : 1;
        double yellow = 0;
        double coarseCamera = robotState.isCoarseCameraMode() ? 1 : 0;

        if (robotState.getBlockColorPreference() == BlockColorPreference.YELLOW) {
            yellow = 1;
            blue = 0;
            red = 0;
        } else if (robotState.getBlockColorPreference() == BlockColorPreference.ANY) {
            yellow = 1;
        }

        limelight.updatePythonInputs(yellow, red, blue, coarseCamera, 0, 0, 0, 0);

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            double[] results = result.getPythonOutput();

            robotState.setBlockLateralFine(results[1] / PIXELS_PER_INCH - WIDTH_RANGE / 2.0);
            robotState.setBlockForwardFine(-(results[2] / PIXELS_PER_INCH - HEIGHT_RANGE / 2.0));
            robotState.setBlockOrientation((-results[3]+270) % 180);
            RobotLog.dd("claw rotation value", String.valueOf(results[3]));
            RobotLog.dd("new claw rotation value", String.valueOf(results[3] + 180));
        }
    }

    @Override
    public void close() {
        limelight.close();
    }
}
