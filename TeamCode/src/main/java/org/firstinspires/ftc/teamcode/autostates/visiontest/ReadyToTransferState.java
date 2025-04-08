package org.firstinspires.ftc.teamcode.autostates.visiontest;


import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.ReadyToTransferAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

import team.techtigers.base.statemachine.ParallelCommandGroupState;

/**
 * A state to bring the intake and dropper into the ready to transfer position
 */
public class ReadyToTransferState extends ParallelCommandGroupState<AutoState> {
    private final RobotState robotState;

    /**
     * Constructor for the ReadyToTransferState
     *
     * @param name       The name of the state
     * @param intake     The intake subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public ReadyToTransferState(String name, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name, 1.5);
        this.robotState = robotState;

        addCommands(
                new ReadyToTransferAction(intake, dropper, robotState)
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (robotState.getDropperState() == DropperState.TRANSFER && robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER) {
            return AutoState.DRIVE_END;
        }
        return AutoState.RUNNING;
    }
}
