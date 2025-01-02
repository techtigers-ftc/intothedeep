package org.firstinspires.ftc.teamcode.autostates;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.statemachine.ParallelCommandGroupState;

/**
 * A state to drop a block
 */
public class IntakeFirstPreloadState extends ParallelCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            IntakeFirstPreloadState.class.getSimpleName();
    private RobotState robotState;

    /**
     * Constructor for the SequentialCommandGroupState
     *
     * @param name The name of the state
     */
    public IntakeFirstPreloadState(String name, IntakeSubsystem intake, RobotState robotState) {
        super(name);
        this.robotState = robotState;
        addCommands(
                //new IntakePrepareToPickupAction(intake, robotState)

        );
    }

    @Override
    public AutoState getCurrentCondition() {
        return isFinished()? AutoState.END_1: AutoState.RUNNING;
    }
}
