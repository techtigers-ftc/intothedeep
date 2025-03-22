package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A class for rumble effects based on a robotState condition
 */
public interface Rumble {

    /**
     * Checks to see if the conditions are met and rumbles accordingly if so
     */
    void updateRumble();

    /**
     * Runs the rumble effect
     */
    void runRumble();
}
