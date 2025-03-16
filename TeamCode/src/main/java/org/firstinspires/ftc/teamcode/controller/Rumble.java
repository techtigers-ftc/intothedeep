package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A class for rumble effects based on a robotState condition
 */
public abstract class Rumble {
    protected final GamepadEx gamepad1;
    protected final GamepadEx gamepad2;
    protected final RobotState robotState;


    /**
     * Constructor for the Rumble class
     * @param gamepad1 The first gamepad to use
     * @param gamepad2 The second gamepad to use
     * @param robotState The robot state to use
     */
    public Rumble(GamepadEx gamepad1, GamepadEx gamepad2, RobotState robotState) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.robotState = robotState;
    }
    /**
     * Constructor for the Rumble class
     * @param gamepad The gamepad to use
     * @param robotState The robot state to use
     */
    public Rumble(GamepadEx gamepad, RobotState robotState) {
        this(gamepad, null, robotState);
    }

    /**
     * Checks to see if the conditions are met and rumbles accordingly if so
     */
    public abstract void updateRumble();

    /**
     * Runs the rumble effect
     */
    protected abstract void runRumble();
}
