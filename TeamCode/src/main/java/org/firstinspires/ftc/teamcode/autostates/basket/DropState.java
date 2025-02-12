package org.firstinspires.ftc.teamcode.autostates.basket;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.statemachine.ParallelCommandGroupState;

/**
 * A state to drop a block
 */
public class DropState extends ParallelCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            DropState.class.getSimpleName();
    private int runCounter;

    /**
     * Constructor for the DropState
     *
     * @param name The name of the state
     * @param dropper The dropper subsystem
     */
    public DropState(String name, DropperSubsystem dropper) {
        super(name);
        runCounter = 0;
        addCommands(
                new DropperOpenAction(dropper, 200)
        );
    }

    /**
     * Initialize the state, incrementing the run counter
     */
    @Override
    public void initialize() {
        runCounter++;
        super.initialize();
    }

    /**
     * Get the current condition of the state
     * @return the current condition of the state based on run counter
     */
    @Override
    public AutoState getCurrentCondition() {
        if (isFinished()) {
            if (runCounter == 1) {
                return AutoState.SAMPLE_PRELOAD_DROP_COMPLETE;
            } else if (runCounter == 2) {
                return AutoState.SAMPLE_1_DROP_COMPLETE;
            } else if(runCounter == 3) {
                return AutoState.SAMPLE_2_DROP_COMPLETE;
            } else if(runCounter == 4) {
                return AutoState.SAMPLE_3_DROP_COMPLETE;
            } else if (runCounter == 5) {
                return AutoState.SAMPLE_4_DROP_COMPLETE;
            } else {
                return AutoState.SAMPLE_5_DROP_COMPLETE;
            }
        }
        return AutoState.RUNNING;
    }
}
