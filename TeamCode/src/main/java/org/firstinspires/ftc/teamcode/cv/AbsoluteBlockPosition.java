package org.firstinspires.ftc.teamcode.cv;

import team.techtigers.core.paths.Waypoint;

/**
 * A class that stores the absolute position of a block on the game field using the robot's
 * orientation and position as well as the block's lateral and forward offset
 */
public class AbsoluteBlockPosition {
    private Waypoint robotPos;

    private double blockLateralInches;
    private double blockForwardInches;
    private double blockOrientation;

    private Waypoint cachedAbsoluteBlockPosition;
    private boolean blockDetected;

    public AbsoluteBlockPosition() {
        robotPos = new Waypoint(0, 0, 0);
        blockLateralInches = 0;
        blockForwardInches = 0;
        blockOrientation = 0;
        cachedAbsoluteBlockPosition = null;
        blockDetected = false;
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
