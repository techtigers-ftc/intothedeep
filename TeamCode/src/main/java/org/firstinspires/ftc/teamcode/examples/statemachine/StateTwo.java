package org.firstinspires.ftc.teamcode.examples.statemachine;

import team.techtigers.base.statemachine.State;

/**
 * Test state for the state machine
 */
public class StateTwo extends State<Condition> {
    /**
     * Constructs a new state
     *
     * @param name The name of the state
     */
    public StateTwo(String name) {
        super(name);
    }

    @Override
    public Condition getCurrentCondition() {
        return Condition.ENDED;
    }

    @Override
    public void initialize() {
        System.out.println("Start state 2");
    }
}
