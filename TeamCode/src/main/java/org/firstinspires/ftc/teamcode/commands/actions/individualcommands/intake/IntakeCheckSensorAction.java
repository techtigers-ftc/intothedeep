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
    private double runCounter;
    private boolean blockDetected;

    /**
     * Constructor for IntakeCheckSensorAction
     * @param robotState the robot state
     * @param command the command to cancel if there is no block
     */
    public IntakeCheckSensorAction(RobotState robotState, CommandBase command) {
        this.robotState = robotState;
        this.command = command;
        runCounter = 0;
        blockDetected = false;
    }

    @Override
    public void initialize() {
        runCounter = 0;
        blockDetected = false;
    }

    @Override
    public void execute() {
        if (robotState.getBlockPosition() == RobotBlockPosition.NONE && robotState.isBreakBeamEnabled()) {
            runCounter++;
        } else {
            blockDetected = true;
        }

        if (runCounter > 3) {
            command.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return blockDetected;
    }
}
