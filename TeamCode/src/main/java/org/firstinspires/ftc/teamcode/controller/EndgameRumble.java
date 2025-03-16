package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Rumbles the controllers for 2 seconds when the endgame starts
 */
public class EndgameRumble extends Rumble {
    private boolean hasRumbled = false;
    /**
     * Constructor for the EndgameRumble class
     * @param gamepad1 The first gamepad to use
     * @param gamepad2 The second gamepad to use
     * @param robotState The state of the robot
     */
    public EndgameRumble(GamepadEx gamepad1, GamepadEx gamepad2, RobotState robotState) {
        super(gamepad1, gamepad2, robotState);
    }

    /**
     * Runs the rumble if is the start of endgame
     */
    @Override
    public void updateRumble() {
        boolean endgameStarted = robotState.getRunTime() / 1000 >= 90;
        if(endgameStarted && !hasRumbled) {
            hasRumbled = true;
            runRumble();
        }
    }

    /**
     * Runs the rumble effect on both controllers for 2 seconds
     */
    @Override
    protected void runRumble() {
        gamepad1.gamepad.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                .addStep(1, 1, 2000)
                .build()
        );
        gamepad2.gamepad.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                .addStep(1, 1, 2000)
                .build()
        );
    }
}
