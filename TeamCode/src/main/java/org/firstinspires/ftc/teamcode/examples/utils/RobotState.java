package org.firstinspires.ftc.teamcode.examples.utils;

import team.techtigers.core.utils.GlobalState;

/**
 * Example implementation of a global state for the robot
 */
public class RobotState extends GlobalState {
    private double intakeHeight;
    private boolean isRunning;

    /**
     * Initializes a new RobotState
     */
    public RobotState() {
        intakeHeight = 0;
        isRunning = false;
    }

    /**
     * @return The height of the intake
     */
    public double getIntakeHeight() {
        return intakeHeight;
    }

    /**
     * Sets the height of the intake
     *
     * @param intakeHeight The height of the intake
     */
    public void setIntakeHeight(double intakeHeight) {
        this.intakeHeight = intakeHeight;
    }

    /**
     * @return Whether the robot is running
     */
    public boolean isRunning() {
        return isRunning;
    }

    /**
     * Sets whether the robot is running
     *
     * @param isRunning Whether the robot is running
     */
    public void setIsRunning(boolean isRunning) {
        this.isRunning = isRunning;
    }
}
