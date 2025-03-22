package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A rumble that reminds the drivers to climb
 */
public class ClimbReminderRumble extends Rumble {
    private boolean hasRumbled = false;

    /**
     * Constructor for the ClimbReminderRumble class
     *
     * @param gamepad1   The first gamepad to use
     * @param gamepad2   The second gamepad to use
     * @param robotState The state of the robot
     */
    public ClimbReminderRumble(GamepadEx gamepad1, GamepadEx gamepad2, RobotState robotState) {
        super(gamepad1, gamepad2, robotState);
    }

    /**
     * Runs two short blips of rumble if the drivers are running out of time to climb
     */
    @Override
    public void updateRumble() {
        boolean needToClimb = robotState.getRunTime() / 1000 >= 105;
        if (needToClimb && !robotState.getIsAscending() && !hasRumbled) {
            runRumble();
            hasRumbled = true;
        }
    }

    /**
     * Runs two long blips
     */
    @Override
    protected void runRumble() {
        gamepad1.gamepad.runRumbleEffect(new com.qualcomm.robotcore.hardware.Gamepad.RumbleEffect.Builder()
                .addStep(1, 1, 500)
                .addStep(0, 0, 500)
                .addStep(1, 1, 500)
                .build()
        );
        gamepad2.gamepad.runRumbleEffect(new com.qualcomm.robotcore.hardware.Gamepad.RumbleEffect.Builder()
                .addStep(1, 1, 500)
                .addStep(0, 0, 500)
                .addStep(1, 1, 500)
                .build()
        );
    }
}
