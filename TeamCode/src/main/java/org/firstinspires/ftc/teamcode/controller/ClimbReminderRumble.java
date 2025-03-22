package org.firstinspires.ftc.teamcode.controller;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A rumble that reminds the drivers to climb
 */
public class ClimbReminderRumble implements ControllerEffect {
    private boolean hasRumbled = false;
    private final RobotState robotState;
    private final GamepadEx gamepad;
    private final Gamepad.RumbleEffect climbRumble = new Gamepad.RumbleEffect.Builder()
            .addStep(1, 1, 500)
            .addStep(0, 0, 500)
            .addStep(1, 1, 500)
            .build();

    /**
     * Constructor for the ClimbReminderRumble class
     *
     * @param gamepad   The gamepad to rumble
     * @param robotState The state of the robot
     */
    public ClimbReminderRumble(GamepadEx gamepad, RobotState robotState) {
        this.gamepad = gamepad;
        this.robotState = robotState;
    }

    /**
     * Runs two short blips of rumble if the drivers are running out of time to climb
     */
    @Override
    public void updateEffect() {
        boolean needToClimb = robotState.getRunTime() / 1000 >= 105;
        if (needToClimb && !robotState.getIsAscending() && !hasRumbled) {
            gamepad.gamepad.runRumbleEffect(climbRumble);
            hasRumbled = true;
        }
    }
}
