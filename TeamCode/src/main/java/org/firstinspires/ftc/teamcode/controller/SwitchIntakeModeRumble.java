package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Rumbles if the driver changes the intake mode (manual or automatic)
 */
public class SwitchIntakeModeRumble implements ControllerEffect {
    private boolean manualIntakeSelected;
    private final GamepadEx gamepad;
    private final RobotState robotState;

    /**
     * Constructor for the SwitchIntakeModeRumble class
     * @param gamepad the gamepad to rumble
     * @param robotState the state of the robot
     */
    public SwitchIntakeModeRumble(GamepadEx gamepad, RobotState robotState) {
        this.gamepad = gamepad;
        this.robotState = robotState;
        manualIntakeSelected = robotState.isManualIntakeSelected();
    }

    /**
     * Rumbles if the driver changes the intake mode
     */
    @Override
    public void updateEffect() {
        if(robotState.isManualIntakeSelected() != manualIntakeSelected){
            runEffect();
        }

        manualIntakeSelected = robotState.isManualIntakeSelected();
    }

    /**
     * Runs a short blip of rumble
     */
    private void runEffect() {
        if(robotState.isManualIntakeSelected()){
            gamepad.gamepad.rumbleBlips(2);
        } else {
            gamepad.gamepad.rumbleBlips(1);
        }
    }
}
