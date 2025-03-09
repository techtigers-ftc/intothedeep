package org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

/**
 * An action that checks if there is a block in the intake
 */
public class IntakeCheckSensorAction extends CommandBase {
    private final RobotState robotState;
    private final CommandBase command;

    public IntakeCheckSensorAction(RobotState robotState, CommandBase command) {
        this.robotState = robotState;
        this.command = command;
    }

    @Override
    public void initialize() {
        if (!robotState.isAuto() && robotState.getBlockPosition() == RobotBlockPosition.NONE && robotState.isBreakBeamEnabled()) {
            command.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
