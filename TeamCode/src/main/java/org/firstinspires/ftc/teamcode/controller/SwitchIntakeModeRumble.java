package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Rumbles if the driver changes the intake mode (manual or automatic)
 */
public class SwitchIntakeModeRumble extends Rumble {
    private boolean manualIntakeSelected;

    /**
     * Constructor for the SwitchIntakeModeRumble class
     * @param manipulatorGamepad the manipulators gamepad
     * @param robotState the state of the robot
     */
    public SwitchIntakeModeRumble(GamepadEx manipulatorGamepad, RobotState robotState) {
        super(manipulatorGamepad, robotState);
        manualIntakeSelected = robotState.isManualIntakeSelected();
    }

    /**
     * Rumbles if the driver changes the intake mode
     */
    @Override
    public void updateRumble() {
        if(robotState.isManualIntakeSelected() != manualIntakeSelected){
            runRumble();
        }

        manualIntakeSelected = robotState.isManualIntakeSelected();
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
