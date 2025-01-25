package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockColorPreference;

/**
 * Command to change the block color preference of the robot.
 */
public class ChangeBlockColorPreferenceCommand extends CommandBase {
    private final RobotState robotState;
    private final GamepadEx gamepad;

    /**
     * Constructor to initialize the command with the robot state.
     *
     * @param robotState the state of the robot
     * @param gamepad    the gamepad to rumble
     */
    public ChangeBlockColorPreferenceCommand(RobotState robotState,
                                             GamepadEx gamepad) {
        this.robotState = robotState;
        this.gamepad = gamepad;
    }

    @Override
    public void initialize() {
        robotState.setBlockColorPreference(robotState.getBlockColorPreference().getNext());

        if (robotState.getBlockColorPreference() == BlockColorPreference.ALLIANCE) {
            gamepad.gamepad.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                    .addStep(1, 1, 200)
            .build());
        } else if (robotState.getBlockColorPreference() == BlockColorPreference.YELLOW) {
            gamepad.gamepad.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                    .addStep(1, 1, 500)
            .build());
        } else {
            gamepad.gamepad.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                    .addStep(1, 1, 1000)
            .build());
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}