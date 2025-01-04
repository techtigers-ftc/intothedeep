package org.firstinspires.ftc.teamcode.autostates;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
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
     * Constructor for the SequentialCommandGroupState
     *
     * @param name The name of the state
     */
    public DropState(String name, DropperSubsystem dropper) {
        super(name);
        runCounter = 0;
        addCommands(
                new DropperOpenAction(dropper, 200)
        );
    }

    @Override
    public void initialize() {
        runCounter++;
        super.initialize();
    }

    @Override
    public AutoState getCurrentCondition() {
        if (isFinished()) {
            if (runCounter == 1) {
                return AutoState.END_1;
            } else if (runCounter == 2) {
                return AutoState.END_2;
            } else if (runCounter == 3){
                return AutoState.END_3;
            }
            return AutoState.SAMPLE_3_DROP_COMPLETE;
        }
        return AutoState.RUNNING;
    }
}
