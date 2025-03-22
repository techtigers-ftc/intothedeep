package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;

import team.techtigers.base.CloseableSubsystem;
import team.techtigers.core.paths.Waypoint;

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
    public static double LIMELIGHT_HEIGHT = 10.25;
    public static double CAMERA_COARSE_ANGLE = 49;
    private final RobotState robotState;
    private final Limelight3A limelight;
    private final AbsoluteBlockPosition absoluteBlockPosition = new AbsoluteBlockPosition();

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
            if ((results.length < 4) || (results[1] == 0 && results[2] == 0 && results[3] == 0)) {
                robotState.setFineBlockDetectionState(BlockDetectionState.NOT_DETECTED);
                robotState.setCoarseBlockDetectionState(BlockDetectionState.NOT_DETECTED);
            } else {
//                RobotLog.dd("VisionDebug", "Limelight's Last Robot Position" + robotPositionForLastUpdate.toString());
//                RobotLog.dd("VisionDebug", "Robot Velocity: " + robotState.getRobotVelocity());
                if (!robotState.isCoarseCameraMode()) {
                    double lateralFine = getCorrectedLateralFine(results[1]);
                    double forwardFine = -(results[2] / PIXELS_PER_INCH - HEIGHT_RANGE / 2.0);
                    double orientation = (results[3] + 180) % 180;

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
                    double forwardCoarse = LIMELIGHT_HEIGHT * Math.tan(Math.toRadians(90 - CAMERA_COARSE_ANGLE + result.getTy()));
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

class AbsoluteBlockPosition {
    private final double CACHE_TIMEOUT = 2000;
    private Waypoint robotPos;
    private final ElapsedTime timer;

    private double blockLateralInches;
    private double blockForwardInches;
    private double blockOrientation;

    private Waypoint cachedAbsoluteBlockPosition;
    private boolean blockDetected;

    /**
     * Constructor for the AbsoluteBlockPosition class
     * Initializes variables and the timer
     */
    public AbsoluteBlockPosition() {
        robotPos = new Waypoint(0, 0, 0);
        blockLateralInches = 0;
        blockForwardInches = 0;
        blockOrientation = 0;
        cachedAbsoluteBlockPosition = null;
        blockDetected = false;
        timer = new ElapsedTime();
    }

    /**
     * Updates the robot's position and the block offsets from the robot
     *
     * @param robotPos the robot position
     * @param blockLateralOffset the block's lateral offset
     * @param blockForwardOffset the block's forward offset
     */
    public void updatePosition(Waypoint robotPos, double blockLateralOffset, double blockForwardOffset, double blockOrientation) {
        this.cachedAbsoluteBlockPosition = null;
        blockDetected = true;
        this.robotPos = robotPos;
        blockLateralInches = blockLateralOffset;
        blockForwardInches = blockForwardOffset;
        this.blockOrientation = blockOrientation;
        timer.reset();
    }

    /**
     * Tells the robot that no block has been recently detected
     */
    public void resetBlockDetection() {
        this.cachedAbsoluteBlockPosition = null;
        blockDetected = false;
    }

    /**
     * Gets the value for if a block has been recently detected
     *
     * @return whether the block has been recently detected
     */
    public boolean isBlockDetected() {
        if (timer.milliseconds() > CACHE_TIMEOUT) {
            blockDetected = false;
        }
        return blockDetected;
    }

    /**
     * Gets the absolute position of the block on the game field using the robot's orientation and
     * position as well as the block's lateral and forward offset
     *
     * @return The absolute position of the block on the game field
     */
    public Waypoint getAbsoluteBlockPosition() {
        if (cachedAbsoluteBlockPosition == null) {
            // Applying a rotational matrix to the block's position
            double blockX = blockForwardInches * Math.cos(robotPos.getHeading()) + blockLateralInches * Math.sin(robotPos.getHeading());
            double blockY = blockForwardInches * Math.sin(robotPos.getHeading()) - blockLateralInches * Math.cos(robotPos.getHeading());
            double blockOrientation = (Math.toDegrees(robotPos.getHeading()) + this.blockOrientation) % 180;
            cachedAbsoluteBlockPosition = robotPos.add(blockX, blockY, blockOrientation);
        }

        return cachedAbsoluteBlockPosition;
    }

    /**
     * Gets the limelight's last known robot position
     *
     * @return the limelight's last known robot position
     */
    public Waypoint getLimelightLastRobotPosition() {
        return this.robotPos;
    }
}