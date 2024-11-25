package org.firstinspires.ftc.teamcode.utils;

import team.techtigers.core.paths.Waypoint;
import team.techtigers.core.utils.GlobalState;

/**
 * Implementation of a global state for the robot
 */
public class RobotState extends GlobalState {
    private Waypoint robotCurrentPose;
    private Waypoint robotCurrentVelocity;
    private double sampleX;
    private double sampleY;
    private double sampleWidth;
    private double sampleHeight;
    private double sampleOrientation;
    private boolean sampleDetected;
    private boolean isPickupComplete;

    /**
     * Initializes a new RobotState
     */
    public RobotState() {
        robotCurrentPose = new Waypoint(0, 0, 0);
        robotCurrentVelocity = new Waypoint(0, 0, 0);

        sampleX = 0;
        sampleY = 0;
        sampleWidth = 0;
        sampleHeight = 0;
        sampleOrientation = 0;
        sampleDetected = false;

        isPickupComplete = false;
    }

    /**
     * @return the current pose of the robot (Inches and Radians)
     */
    public Waypoint getRobotCurrentPose() {
        return robotCurrentPose;
    }

    /**
     * Sets the current pose of the robot (Inches and Radians)
     *
     * @param robotPose the current pose of the robot
     */
    public void setRobotPose(Waypoint robotPose) {
        robotCurrentPose = robotPose;
    }

    /**
     * @return the current velocity of the robot (Inches and Radians)
     */
    public Waypoint getRobotVelocity() {
        return robotCurrentVelocity;
    }

    /**
     * Sets the current velocity of the robot (Inches and Radians)
     *
     * @param robotVelocity the current velocity of the robot
     */
    public void setRobotVelocity(Waypoint robotVelocity) {
        robotCurrentVelocity = robotVelocity;
    }

    /**
     * @return the x value of the sample
     */
    public double getSampleX() {
        return sampleX;
    }

    /**
     * Sets the x value of the sample
     *
     * @param sampleX the x value of the sample
     */
    public void setSampleX(double sampleX) {
        this.sampleX = sampleX;
    }

    /**
     * @return the y value of the sample
     */
    public double getSampleY() {
        return sampleY;
    }

    /**
     * Sets the y value of the sample
     *
     * @param sampleY the y value of the sample
     */
    public void setSampleY(double sampleY) {
        this.sampleY = sampleY;
    }

    /**
     * @return the orientation of the sample
     */
    public double getSampleOrientation() {
        return sampleOrientation;
    }

    /**
     * Sets the orientation of the sample
     *
     * @param sampleOrientation the orientation of the sample
     */
    public void setSampleOrientation(double sampleOrientation) {
        this.sampleOrientation = sampleOrientation;
    }

    /**
     * @return whether the sample is detected
     */
    public boolean isSampleDetected() {
        return sampleDetected;
    }

    /**
     * Sets whether the sample is detected
     *
     * @param sampleDetected whether the sample is detected
     */
    public void setSampleDetected(boolean sampleDetected) {
        this.sampleDetected = sampleDetected;
    }

    /**
     * @return the width of the sample
     */
    public double getSampleWidth() {
        return sampleWidth;
    }

    /**
     * Sets the width of the sample
     *
     * @param sampleWidth the width of the sample
     */
    public void setSampleWidth(double sampleWidth) {
        this.sampleWidth = sampleWidth;
    }

    /**
     * @return the height of the sample
     */
    public double getSampleHeight() {
        return sampleHeight;
    }

    /**
     * Sets the height of the sample
     *
     * @param sampleHeight the height of the sample
     */
    public void setSampleHeight(double sampleHeight) {
        this.sampleHeight = sampleHeight;
    }

    /**
     * @return whether or not the sample has been picked up
     */
    public boolean isPickupComplete() {
        return isPickupComplete;
    }

    /**
     * sets whether or not the sample has been picked up
     *
     * @param pickupComplete boolean value to set
     */
    public void setPickupComplete(boolean pickupComplete) {
        isPickupComplete = pickupComplete;
    }
}
