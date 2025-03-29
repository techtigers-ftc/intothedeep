package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.AbsoluteBlockPosition;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;

import team.techtigers.base.CloseableSubsystem;

/**
 * A subsystem which saves a detected sample's attributes into RobotState
 */
@Config
public class LimelightSubsystem extends CloseableSubsystem {
    public static final double SLIDES_OFFSET = 7; // forward distance from center of robot to slides
    public static final double LIMELIGHT_COARSE_OFFSET = 2; // forward distance from end of slides to limelight lens in coarse
    public static final double LIMELIGHT_FINE_OFFSET = 5.5; // forward distance from end of slides to limelight lens in fine
    private static final int BLOCK_CACHE_LIMIT = 3;

    private static final double WIDTH_RANGE = 6; // TODO: Tune properly
    private static final double HEIGHT_RANGE = 5.6;
    private static final double PIXELS_PER_INCH = 94.5;

    // Limelight fine horizontal linear equation parameters
    private static final double LATERAL_FINE_VERTICAL_COMPRESSION = 0.0110083;
    private static final double LATERAL_FINE_VERTICAL_SHIFT = -3.55128;
    public static double LIMELIGHT_HEIGHT_COARSE = 10.25;
    public static double LIMELIGHT_HEIGHT_FINE = 8.5;
    public static double CAMERA_FINE_ANGLE = 22;
    public static double CAMERA_COARSE_ANGLE = 49;
    private final RobotState robotState;
    private final Limelight3A limelight;
    private final AbsoluteBlockPosition absoluteBlockPosition;

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
        absoluteBlockPosition = new AbsoluteBlockPosition(robotState);
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
        double coarseCamera = robotState.isCoarseCameraMode() ? 0 : 1;

        if (robotState.getBlockColorPreference() == BlockColorPreference.YELLOW) {
            yellow = 1;
            blue = 0;
            red = 0;
        } else if (robotState.getBlockColorPreference() == BlockColorPreference.ANY) {
            yellow = 1;
        }

        limelight.updatePythonInputs(yellow, red, blue, 1, 0, 0, 0, 0);

        LLResult result = limelight.getLatestResult();

        if (result != null) {
            double[] results = result.getPythonOutput();
            if ((results.length < 4) || (results[1] == 0 && results[2] == 0 && results[3] == 0)) {
                robotState.setFineBlockDetectionState(BlockDetectionState.NOT_DETECTED);
                robotState.setCoarseBlockDetectionState(BlockDetectionState.NOT_DETECTED);
            } else {
                if (!robotState.isCoarseCameraMode()) {
//                    double lateralFine = getCorrectedLateralFine(results[1]);
//                    double forwardFine = -(results[2] / PIXELS_PER_INCH - HEIGHT_RANGE / 2.0);
                    double orientation = (results[3] + 180) % 180;

                    double forwardFine = LIMELIGHT_HEIGHT_FINE * Math.tan(Math.toRadians(CAMERA_FINE_ANGLE + result.getTy()));
                    double lateralFine = forwardFine * Math.tan(Math.toRadians(result.getTx()));
                    RobotLog.dd("limelight", "tx: %f", result.getTx());
                    RobotLog.dd("limelight", "ty: %f", result.getTy());

                    robotState.setFineBlockDetectionState(BlockDetectionState.DETECTED);
                    robotState.setCoarseBlockDetectionState(BlockDetectionState.NOT_DETECTED);
                    robotState.setBlockLateralFine(lateralFine);
                    robotState.setBlockForwardFine(forwardFine);
                    robotState.setBlockOrientation(orientation);

                    // Saving the block's relative position whenever a block is seen
                    absoluteBlockPosition.updatePosition(
                            robotState.getRobotCurrentPose(),
                            lateralFine,
                            forwardFine + SLIDES_OFFSET + LIMELIGHT_FINE_OFFSET + robotState.getIntakeSlidePosition(),
                            robotState.getBlockOrientation()
                    );
                    robotState.setAbsoluteBlockPosition(absoluteBlockPosition.getAbsoluteBlockPosition());
                } else {
                    double forwardCoarse = LIMELIGHT_HEIGHT_COARSE * Math.tan(Math.toRadians(90 - CAMERA_COARSE_ANGLE + result.getTy()));
                    double lateralCoarse = forwardCoarse * Math.tan(Math.toRadians(result.getTx()));

                    robotState.setCoarseBlockDetectionState(BlockDetectionState.DETECTED);
                    robotState.setFineBlockDetectionState(BlockDetectionState.NOT_DETECTED);
                    robotState.setBlockForwardCoarse(forwardCoarse);
                    robotState.setBlockLateralCoarse(lateralCoarse);

                    // Saving the block's relative position whenever a block is seen
                    absoluteBlockPosition.updatePosition(
                            robotState.getRobotCurrentPose(),
                            lateralCoarse,
                            forwardCoarse + SLIDES_OFFSET + LIMELIGHT_COARSE_OFFSET + robotState.getIntakeSlidePosition(),
                            robotState.getBlockOrientation()
                    );

                    robotState.setAbsoluteBlockPosition(absoluteBlockPosition.getAbsoluteBlockPosition());
                }
            }
        }
        robotState.setBlockDetected(absoluteBlockPosition.isBlockDetected());
    }

    @Override
    public void close() {
        limelight.close();
    }
}