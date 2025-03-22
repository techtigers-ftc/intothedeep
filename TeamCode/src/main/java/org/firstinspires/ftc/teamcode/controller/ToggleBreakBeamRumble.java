package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Rumbles the controllers when the driver toggles the break beam
 */
public class ToggleBreakBeamRumble extends Rumble{
    private boolean breakBeamWasEnabled;

    /**
     * Constructs a ToggleBreakBeamRumble
     * @param manipulatorGamepad the manipulators gamepad
     * @param robotState the state of the robot
     */
    public ToggleBreakBeamRumble(GamepadEx manipulatorGamepad, RobotState robotState) {
        super(manipulatorGamepad, robotState);
        breakBeamWasEnabled = robotState.isBreakBeamEnabled();
    }

    /**
     * Rumbles if the driver toggles the break beam
     */
    @Override
    public void updateRumble() {
        if(robotState.isBreakBeamEnabled() != breakBeamWasEnabled){
            runRumble();
        }

        breakBeamWasEnabled = robotState.isBreakBeamEnabled();
    }

    /**
     * Runs a short blip of rumble
     */
    @Override
    protected void runRumble() {
        if(robotState.isBreakBeamEnabled()){
            gamepad1.gamepad.rumbleBlips(1);
        } else {
            gamepad1.gamepad.rumbleBlips(2);
        }
    }
}
