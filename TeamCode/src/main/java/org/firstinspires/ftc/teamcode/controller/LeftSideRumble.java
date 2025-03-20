package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.RobotState;

public class LeftSideRumble extends Rumble{
    private GamepadEx manipulatorGamepad;

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
