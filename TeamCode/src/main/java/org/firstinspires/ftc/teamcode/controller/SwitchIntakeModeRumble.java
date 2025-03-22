package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.utils.RobotState;

public class SwitchIntakeModeRumble extends Rumble {

    public SwitchIntakeModeRumble(GamepadEx manipulatorGamepad, RobotState robotState) {
        super(manipulatorGamepad, robotState);
    }

    @Override
    public void updateRumble() {
        if(gamepad1.wasJustPressed(GamepadKeys.Button.START)){
            runRumble();
        }
    }

    @Override
    protected void runRumble() {
        if(robotState.isManualIntakeSelected()){
            gamepad1.gamepad.rumbleBlips(2);
        } else {
            gamepad1.gamepad.rumbleBlips(1);
        }
    }
}
