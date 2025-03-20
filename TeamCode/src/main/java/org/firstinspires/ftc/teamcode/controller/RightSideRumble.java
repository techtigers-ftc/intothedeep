package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.RobotState;

public class RightSideRumble extends Rumble{
    private GamepadEx manipulatorGamepad;

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
