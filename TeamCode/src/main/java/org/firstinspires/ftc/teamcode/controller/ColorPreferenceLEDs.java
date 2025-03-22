package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Displays the color preference on the gamepad LEDs
 */
public class ColorPreferenceLEDs implements ControllerEffect {
    private final RobotState robotState;
    private final GamepadEx gamepad;

    /**
     * Constructor for the ColorPreferenceLEDs class
     *
     * @param gamepad  The gamepad to display the color preference on
     * @param robotState The state of the robot
        */
    public ColorPreferenceLEDs(GamepadEx gamepad, RobotState robotState) {
        this.gamepad = gamepad;
        this.robotState = robotState;
    }

    @Override
    public void updateEffect() {
        if(!robotState.isAuto()){

        }
    }

    @Override
    public void runEffect() {

    }
}
