package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.utils.RobotState;

public class ToggleBreakBeamRumble extends Rumble{

    public ToggleBreakBeamRumble(GamepadEx manipulatorGamepad, RobotState robotState) {
        super(manipulatorGamepad, robotState);
    }

    @Override
    public void updateRumble() {
        if(gamepad1.wasJustPressed(GamepadKeys.Button.Y)){
            runRumble();
        }

    }

    @Override
    protected void runRumble() {
        if(robotState.isBreakBeamEnabled()){
            gamepad1.gamepad.rumbleBlips(1);
        } else {
            gamepad1.gamepad.rumbleBlips(2);
        }
    }
}
