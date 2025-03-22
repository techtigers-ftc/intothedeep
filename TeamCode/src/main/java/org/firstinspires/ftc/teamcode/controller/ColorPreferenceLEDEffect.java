package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Displays the color preference on the gamepad LEDs
 */
public class ColorPreferenceLEDEffect implements ControllerEffect {
    private final RobotState robotState;
    private final GamepadEx gamepad;

    /**
     * Constructor for the ColorPreferenceLEDs class
     *
     * @param gamepad    The gamepad to display the color preference on
     * @param robotState The state of the robot
     */
    public ColorPreferenceLEDEffect(GamepadEx gamepad, RobotState robotState) {
        this.gamepad = gamepad;
        this.robotState = robotState;
    }

    /**
     * Updates the effect on the gamepad LEDs based on the color preference
     */
    @Override
    public void updateEffect() {
        switch (robotState.getBlockColorPreference()) {
            case ALLIANCE:
                if(robotState.isBlue()){
                    gamepad.gamepad.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
                } else {
                    gamepad.gamepad.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                }
                break;
            case YELLOW:
                gamepad.gamepad.setLedColor(1, 0.8, 0, Gamepad.LED_DURATION_CONTINUOUS);
                break;
            case ANY:
                gamepad.gamepad.setLedColor(1, 1, 1, Gamepad.LED_DURATION_CONTINUOUS);
                break;
        }
    }


}