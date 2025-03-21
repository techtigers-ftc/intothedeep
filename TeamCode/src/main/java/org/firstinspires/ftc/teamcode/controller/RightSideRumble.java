package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 Class for a rumble that only rumbles on the right side of the controller
 **/
public class RightSideRumble extends Rumble{
    private GamepadEx manipulatorGamepad;

    /**
     Constructor for a rumble that only rumbles on the right side of the controller
     @param manipulatorGamepad: Manipulator gamepad
     @param robotState: State of the robot
     **/
    public RightSideRumble(GamepadEx manipulatorGamepad, RobotState robotState) {
        super(manipulatorGamepad, robotState);
    }

    @Override
    public void updateRumble() {
        runRumble();
    }

    @Override
    protected void runRumble() {
        manipulatorGamepad.gamepad.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                .addStep(0, 1, 500)
                .addStep(0, 0, 500)
                .addStep(0, 1, 500)
                .build());
    }
}
