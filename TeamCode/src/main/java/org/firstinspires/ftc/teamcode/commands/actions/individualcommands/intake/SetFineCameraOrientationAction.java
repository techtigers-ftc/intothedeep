package org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * A command to set the fine camera orientation to the detected orientation
 */
public class SetFineCameraOrientationAction extends InstantCommand {
    private final RobotState robotState;

    /**
     * Creates a new SetFineCameraOrientationAction
     *
     * @param robotState the robot state
     */
    public SetFineCameraOrientationAction(RobotState robotState) {
        this.robotState = robotState;
    }

    @Override
    public void initialize() {
        robotState.setDetectedFineBlockOrientation(robotState.getBlockOrientation());
    }
}
