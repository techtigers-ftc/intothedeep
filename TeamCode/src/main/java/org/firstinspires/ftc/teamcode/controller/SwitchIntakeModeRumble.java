package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Rumbles if the driver changes the intake mode (manual or automatic)
 */
public class SwitchIntakeModeRumble extends Rumble {

    /**
     * Constructor for the SwitchIntakeModeRumble class
     * @param manipulatorGamepad the manipulators gamepad
     * @param robotState the state of the robot
     */
    public SwitchIntakeModeRumble(GamepadEx manipulatorGamepad, RobotState robotState) {
        super(manipulatorGamepad, robotState);
    }

    /**
     * Rumbles if the driver changes the intake mode
     */
    @Override
    public void updateRumble() {
        if(gamepad1.wasJustPressed(GamepadKeys.Button.START)){
            runRumble();
        }
    }

    /**
     * Runs a short blip of rumble
     */
    @Override
    protected void runRumble() {
        if(robotState.isManualIntakeSelected()){
            gamepad1.gamepad.rumbleBlips(2);
        } else {
            gamepad1.gamepad.rumbleBlips(1);
        }
    }
}
