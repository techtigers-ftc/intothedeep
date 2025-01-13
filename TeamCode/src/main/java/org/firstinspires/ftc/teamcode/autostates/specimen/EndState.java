package org.firstinspires.ftc.teamcode.autostates.specimen;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.statemachine.ParallelCommandGroupState;

/**
 * A state to drop a block
 */
public class EndState extends ParallelCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            EndState.class.getSimpleName();

    /**
     * Constructor for the DropState
     *
     * @param name The name of the state
     */
    public EndState(String name) {
        super(name);
    }

    /**
     * Get the current condition of the state
     * @return the current condition of the state based on run counter
     */
    @Override
    public AutoState getCurrentCondition() {
        return AutoState.RUNNING;
    }
}
