package org.firstinspires.ftc.teamcode.autostates;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.statemachine.ParallelCommandGroupState;

public abstract class TimeoutStateBase extends ParallelCommandGroupState<AutoState> {
    private final ElapsedTime timer;
    private final double timeout;

    /**
     * Constructor for the SequentialCommandGroupState
     *
     * @param name The name of the state
     * @param timeout the max time of the state in milliseconds
     */
    public TimeoutStateBase(String name, double timeout) {
        super(name);
        this.timeout = timeout;
        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public AutoState getCurrentCondition() {
        if (timer.milliseconds() > timeout && timeout > 0) {
            return AutoState.TIMEOUT;
        } else {
            return AutoState.RUNNING;
        }
    }
}

