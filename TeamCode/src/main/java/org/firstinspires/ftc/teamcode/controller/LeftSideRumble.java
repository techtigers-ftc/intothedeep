package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
  Class for rumble that rumbles only on the left side of the controller
 **/
public class LeftSideRumble extends Rumble{
    private GamepadEx manipulatorGamepad;

    /**
     Class for rumble that rumbles only on the left side of the controller
     @param manipulatorGamepad: Manipulator gamepad
     @param robotState: State of the robot
     **/
    public LeftSideRumble(GamepadEx manipulatorGamepad, RobotState robotState) {
        super(manipulatorGamepad, robotState);
    }

    @Override
    public void updateRumble() {
        runRumble();
    }

    @Override
    protected void runRumble() {
        manipulatorGamepad.gamepad.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                .addStep(1, 0, 500)
                .addStep(0, 0, 500)
                .addStep(1, 0, 500)
                .build());
    }
}
