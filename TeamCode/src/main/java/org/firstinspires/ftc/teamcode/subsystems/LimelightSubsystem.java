package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem which saves a detected sample's attributes into RobotState
 */
@Config
public class LimelightSubsystem extends CloseableSubsystem {
    private static final int BLOCK_CACHE_LIMIT = 3;
    private final static double LIMELIGHT_HEIGHT = 10.25;
    private final static double LIMELIGHT_X_OFFSET = 4;
    private static final double LIMELIGHT_INTAKE_OFFSET = 8;
    private static final double LIMELIGHT_DOWNWARD_ANGLE = 25;

    private static final double WIDTH_RANGE = 6; // TODO: Tune properly
    private static final double HEIGHT_RANGE = 5.6;
    private static final double PIXELS_PER_INCH = 94.5;
    private static final double CAMERA_COARSE_OFFSET = 41;

    // Limelight fine horizontal linear equation parameters
    private static final double LATERAL_FINE_VERTICAL_COMPRESSION = 0.0110083;
    private static final double LATERAL_FINE_VERTICAL_SHIFT = -3.55128;

    private final RobotState robotState;
    private final Limelight3A limelight;

    // Lateral bounds
    private double lateralLowerBound = -5;
    private double lateralUpperBound = 1;
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
        limelight.pipelineSwitch(3);
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

    /**
     * Gets the corrected lateral fine distance of the block from the robot
     *
     * @param lateralFine the raw lateral fine distance of the block from the limelight (in pixels)
     * @return the corrected lateral fine distances
     */
    private double getCorrectedLateralFine(double lateralFine) {
        return lateralFine * LATERAL_FINE_VERTICAL_COMPRESSION + LATERAL_FINE_VERTICAL_SHIFT;
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
            if (results[1] == 0 && results[2] == 0 && results[3] == 0) {
                robotState.setFineBlockDetectionState(BlockDetectionState.NOT_DETECTED);
                robotState.setCoarseBlockDetectionState(BlockDetectionState.NOT_DETECTED);
//                robotState.setDebugColor(Color.BLUE);
            } else {
                if (!robotState.isCoarseCameraMode()) {
                    robotState.setFineBlockDetectionState(BlockDetectionState.DETECTED);
                    robotState.setCoarseBlockDetectionState(BlockDetectionState.NOT_DETECTED);
//                  robotState.setDebugColor(Color.BLACK);
                    robotState.setBlockLateralFine(getCorrectedLateralFine(results[1]));
                    robotState.setBlockForwardFine(-(results[2] / PIXELS_PER_INCH - HEIGHT_RANGE / 2.0));
                    robotState.setBlockOrientation((results[3] + 180) % 180);
                } else {
                    robotState.setCoarseBlockDetectionState(BlockDetectionState.DETECTED);
                    robotState.setFineBlockDetectionState(BlockDetectionState.NOT_DETECTED);
                    double forwardCoarse = LIMELIGHT_HEIGHT * Math.tan(Math.toRadians(90 - CAMERA_COARSE_OFFSET + result.getTy()));
                    double lateralCoarse = forwardCoarse * Math.tan(Math.toRadians(result.getTx()));
                    robotState.setBlockForwardCoarse(forwardCoarse);
                    robotState.setBlockLateralCoarse(lateralCoarse);
                }
            }
        }
    }

    @Override
    public void close() {
        limelight.close();
    }
}
