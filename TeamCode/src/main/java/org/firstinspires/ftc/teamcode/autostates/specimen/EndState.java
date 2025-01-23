package org.firstinspires.ftc.teamcode.autostates.specimen;

import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import team.techtigers.base.statemachine.ParallelCommandGroupState;

/**
 * An end state for the state machine to end the autonomous
 */
public class EndState extends ParallelCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            EndState.class.getSimpleName();

    /**
     * Constructor for the EndState
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
