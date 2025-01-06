package org.firstinspires.ftc.teamcode.autostates.specimen;

import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.statemachine.CommandState;

/**
 * A placeholder class for the end state of an autonomous command
 */
public class EndState extends CommandState<AutoState> {

    /**
     * Constructor for the CommandState
     *
     * @param name The name of the state
     */
    public EndState(String name) {
        super(name);
    }

    @Override
    public AutoState getCurrentCondition() {
        return AutoState.RUNNING;
    }
}
