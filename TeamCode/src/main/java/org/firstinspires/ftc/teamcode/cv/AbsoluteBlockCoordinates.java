package org.firstinspires.ftc.teamcode.cv;

import team.techtigers.core.paths.Waypoint;

/**
 * A class that stores the absolute position of a block on the game field using the robot's
 * orientation and position as well as the block's lateral and forward offset
 */
public class AbsoluteBlockCoordinates {
    private double robotXPos;
    private double robotYPos;
    private double robotOrientation; // in radians

    private double blockLateralInches;
    private double blockForwardInches;
    private double blockOrientation;

    private boolean hasBeenDetected = false;

    /**
     * Set the robot position
     *
     * @param robotPos the robot position
     */
    public void setRobotPosition(Waypoint robotPos) {
        this.robotXPos = robotPos.getX();
        this.robotYPos = robotPos.getY();
        this.robotOrientation = robotPos.getHeading();
    }

    /**
     * Sets the block's lateral offset to the robot
     *
     * @param blockLateralInches the block's lateral offset to the robot
     */
    public void setBlockLateralInches(double blockLateralInches) {
        hasBeenDetected = true;
        this.blockLateralInches = blockLateralInches;
    }

    /**
     * Sets the block's lateral offset to the robot
     *
     * @param blockForwardInches the block's lateral offset to the robot
     */
    public void setBlockForwardInches(double blockForwardInches) {
        hasBeenDetected = true;
        this.blockForwardInches = blockForwardInches;
    }

    /**
     * Set's the block's orientation detected by the camera
     *
     * @param blockOrientation the block's orientation detected by the camera
     */
    public void setBlockOrientation(double blockOrientation) {
        hasBeenDetected = true;
        this.blockOrientation = blockOrientation;
    }

    /**
     * Tells the robot that no block has been recently detected
     */
    public void resetBlockHasBeenDetected() {
        hasBeenDetected = false;
    }

    /**
     * Gets the value for if a block has been recently detected
     *
     * @return whether the block has been recently detected
     */
    public boolean getBlockHasBeenDetected() {
        return hasBeenDetected;
    }

    /**
     * Gets the absolute position of the block on the game field using the robot's orientation and
     * position as well as the block's lateral and forward offset
     *
     * @return The absolute position of the block on the game field
     */
    public Waypoint getBlockAbsolutePosition() {
        // Applying a rotational matrix to the block's position
        double blockX = blockForwardInches * Math.cos(robotOrientation) + blockLateralInches * Math.sin(robotOrientation);
        double blockY = blockForwardInches * Math.sin(robotOrientation) - blockLateralInches * Math.cos(robotOrientation);

        // Returning the translated coordinates + the robot's coordinates
        return new Waypoint(robotXPos + blockX, robotYPos + blockY, blockOrientation);
    }

    /**
     * Gets the limelight's last known robot position
     *
     * @return the limelight's last known robot position
     */
    public Waypoint getLimelightLastRobotPosition() {
        return new Waypoint(robotXPos, robotYPos, robotOrientation);
    }
}
