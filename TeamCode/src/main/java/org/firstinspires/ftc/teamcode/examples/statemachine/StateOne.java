package org.firstinspires.ftc.teamcode.examples.statemachine;

import com.qualcomm.robotcore.util.ElapsedTime;

import team.techtigers.base.statemachine.State;

/**
 * Test state for the state machine
 */
public class StateOne extends State<Condition> {
    private final ElapsedTime timer;

    /**
     * Constructs a new state
     *
     * @param name The name of the state
     */
    public StateOne(String name) {
        super(name);
        timer = new ElapsedTime();
    }

    @Override
    public Condition getCurrentCondition() {
        return timer.seconds() > 2 ? Condition.ENDED : Condition.RUNNING;
    }

    @Override
    public void initialize() {
        System.out.println("Start state 1");
        timer.reset();
    }
}
