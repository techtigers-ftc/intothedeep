package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Rumbles if the driver changes the intake mode (manual or automatic)
 */
public class SwitchIntakeModeRumble implements ControllerEffect {
    private boolean manualIntakeSelected;
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

    /**
     * Constructor for the SwitchIntakeModeRumble class
     *
     * @param gamepad    the gamepad to rumble
     * @param robotState the state of the robot
     */
    public SwitchIntakeModeRumble(GamepadEx gamepad, RobotState robotState) {
        this.gamepad = gamepad;
        this.robotState = robotState;
        manualIntakeSelected = robotState.isManualIntakeSelected();
    }

    /**
     * Rumbles if the driver changes the intake mode
     */
    @Override
    public void updateEffect() {
        if (robotState.isManualIntakeSelected() != manualIntakeSelected) {
            runEffect();
        }

        manualIntakeSelected = robotState.isManualIntakeSelected();
    }

    /**
     * Runs a short blip of rumble
     */
    private void runEffect() {
        if (robotState.isManualIntakeSelected()) {
            gamepad.gamepad.runRumbleEffect(enableEffect);
        } else {
            gamepad.gamepad.runRumbleEffect(disableEffect);

        }
    }
}
