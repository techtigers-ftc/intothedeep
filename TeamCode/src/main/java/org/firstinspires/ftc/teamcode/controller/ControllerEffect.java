package org.firstinspires.ftc.teamcode.controller;

/**
 * A class for rumble effects based on a robotState condition
 */
public interface ControllerEffect {

    /**
     * Checks to see if the conditions are met and rumbles accordingly if so
     */
    void updateEffect();

    /**
     * Runs the rumble effect
     */
    void runEffect();
}
