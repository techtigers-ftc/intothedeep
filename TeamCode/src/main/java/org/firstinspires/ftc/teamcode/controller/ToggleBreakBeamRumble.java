package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Rumbles the controllers when the driver toggles the break beam
 */
public class ToggleBreakBeamRumble implements ControllerEffect {
    private final GamepadEx gamepad;
    private final RobotState robotState;
    private final Gamepad.RumbleEffect enableEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(1, 1, 100)
            .addStep(0, 0, 100)
            .addStep(1, 1, 100)
            .build();
    private final Gamepad.RumbleEffect disableEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(1, 1, 100)
            .build();
    private boolean breakBeamWasEnabled;

    /**
     * Constructs a ToggleBreakBeamRumble
     *
     * @param gamepad    the manipulators gamepad
     * @param robotState the state of the robot
     */
    public ToggleBreakBeamRumble(GamepadEx gamepad, RobotState robotState) {
        this.gamepad = gamepad;
        this.robotState = robotState;
        breakBeamWasEnabled = robotState.isBreakBeamEnabled();
    }

    /**
     * Rumbles if the driver toggles the break beam
     */
    @Override
    public void updateEffect() {
        if (robotState.isBreakBeamEnabled() != breakBeamWasEnabled) {
            runEffect();
        }

        breakBeamWasEnabled = robotState.isBreakBeamEnabled();
    }

    /**
     * Runs a short blip of rumble
     */
    private void runEffect() {
        if (robotState.isBreakBeamEnabled()) {
            gamepad.gamepad.runRumbleEffect(enableEffect);
        } else {
            gamepad.gamepad.runRumbleEffect(disableEffect);
        }
    }
}
