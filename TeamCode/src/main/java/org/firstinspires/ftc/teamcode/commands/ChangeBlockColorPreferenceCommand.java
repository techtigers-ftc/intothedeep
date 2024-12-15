package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Command to change the block color preference of the robot.
 */
public class ChangeBlockColorPreferenceCommand extends CommandBase {
    private final RobotState robotState;

    /**
     * Constructor to initialize the command with the robot state.
     *
     * @param robotState the state of the robot
     */
    public ChangeBlockColorPreferenceCommand(RobotState robotState) {
        this.robotState = robotState;
    }

    @Override
    public void initialize() {
        robotState.setBlockColorPreference(robotState.getBlockColorPreference().getNext());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}