package org.firstinspires.ftc.teamcode.utils;

/**
 * Singleton class for constants for the robot, such as alliance color and game period
 **/
public class GlobalConstants {
    private static GlobalConstants instance;
    public final boolean isAuto;
    public final boolean isRed;

    private GlobalConstants(boolean isAuto, boolean isRed) {
        this.isAuto = isAuto;
        this.isRed = isRed;
    }

    /**
     * Initializes the GlobalConstants instance
     * This should be run at the beginning of the opmode
     *
     * @param isAuto whether or not the robot is in autonomous mode
     * @param isRed  whether or not the robot is on the red alliance
     */
    public static void initialize(boolean isAuto, boolean isRed) {
        instance = new GlobalConstants(isAuto, isRed);
    }

    /**
     * @return the instance of GlobalConstants
     */
    public static GlobalConstants getInstance() {
        if (instance == null) {
            throw new IllegalStateException("GlobalConstants not initialized");
        }
        return instance;
    }

}
