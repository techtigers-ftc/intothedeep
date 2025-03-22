package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Rumbles the controllers for 2 seconds when the endgame starts
 */
public class EndgameRumble implements ControllerEffect {
    private boolean hasRumbled = false;
    private final RobotState robotState;
    private final GamepadEx gamepad;
    private final Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(1, 1, 2000)
            .build();
    /**
     * Constructor for the EndgameRumble class
     * @param gamepad the gamepad to rumble
     * @param robotState The state of the robot
     */
    public EndgameRumble(GamepadEx gamepad, RobotState robotState) {
        this.gamepad = gamepad;
        this.robotState = robotState;
    }

    /**
     * Runs the rumble if is the start of endgame
     */
    @Override
    public void updateEffect() {
        boolean endgameStarted = robotState.getRunTime() / 1000 >= 90;
        if(endgameStarted && !hasRumbled) {
            hasRumbled = true;
            gamepad.gamepad.runRumbleEffect(rumbleEffect);
        }
    }
}
