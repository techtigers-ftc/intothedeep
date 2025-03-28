package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import team.techtigers.core.paths.Waypoint;

/**
 * Class to calculate the absolute position of a block based on the robot's position and the block's offsets
 */
public class AbsoluteBlockPosition {
    private final double CACHE_TIMEOUT = 2000;
    private final double TELE_CACHE_TIMEOUT = 1000;
    private Waypoint robotPos;
    private final ElapsedTime timer;

    private double blockLateralInches;
    private double blockForwardInches;
    private double blockOrientation;

    private Waypoint cachedAbsoluteBlockPosition;
    private boolean blockDetected;
    private RobotState robotState;

    /**
     * Constructor for the AbsoluteBlockPosition class
     * Initializes variables and the timer
     */
    public AbsoluteBlockPosition(RobotState robotState) {
        robotPos = new Waypoint(0, 0, 0);
        blockLateralInches = 0;
        blockForwardInches = 0;
        blockOrientation = 0;
        this.robotState = robotState;
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
        double cacheTimeout = robotState.isAuto()? CACHE_TIMEOUT : TELE_CACHE_TIMEOUT;
        if (timer.milliseconds() > cacheTimeout) {
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
            double blockOrientation =
                    (Math.toDegrees(robotPos.getHeading()) - this.blockOrientation + 450) % 180;
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
