package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Rumbles the controllers when the driver toggles the break beam
 */
public class ToggleBreakBeamRumble implements ControllerEffect {
    private boolean breakBeamWasEnabled;
    private final GamepadEx gamepad;
    private final RobotState robotState;

    /**
     * Constructs a ToggleBreakBeamRumble
     * @param gamepad the manipulators gamepad
     * @param robotState the state of the robot
     */
    public ToggleBreakBeamRumble(GamepadEx gamepad, RobotState robotState) {
        this.gamepad = gamepad;
        this.robotState = robotState;
        breakBeamWasEnabled = robotState.isBreakBeamEnabled();
    }

    /**
     * Rumbles if the driver toggles the break beam
     */
    @Override
    public void updateEffect() {
        if(robotState.isBreakBeamEnabled() != breakBeamWasEnabled){
            runEffect();
        }

        breakBeamWasEnabled = robotState.isBreakBeamEnabled();
    }

    /**
     * Runs a short blip of rumble
     */
    @Override
    public void runEffect() {
        if(robotState.isBreakBeamEnabled()){
            gamepad.gamepad.rumbleBlips(1);
        } else {
            gamepad.gamepad.rumbleBlips(2);
        }
    }
}
