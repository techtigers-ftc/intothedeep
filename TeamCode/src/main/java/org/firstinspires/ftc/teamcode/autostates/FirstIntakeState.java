package org.firstinspires.ftc.teamcode.autostates;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToTransferAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A state to drop a block
 */
public class FirstIntakeState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            FirstIntakeState.class.getSimpleName();
    private RobotState robotState;

    /**
     * Constructor for the SequentialCommandGroupState
     *
     * @param name The name of the state
     */
    public FirstIntakeState(String name, IntakeSubsystem intake, RobotState robotState) {
        super(name);
        this.robotState = robotState;
        addCommands(
                new IntakePrepareToPickupAction(intake, robotState, 19),
                new IntakeReadyToPickupAction(intake, robotState, 78),
                new IntakePrepareToTransferAction(intake, robotState),
                new IntakeReadyToTransferAction(intake, robotState)

        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if(robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER){
            return AutoState.END_1;
        }
        return AutoState.RUNNING;
    }
}
