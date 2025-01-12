package org.firstinspires.ftc.teamcode.autostates.specimen;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperFrontSlapAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.statemachine.ParallelCommandGroupState;

/**
 * A state to drop a block
 */
public class DropSpecimenState extends ParallelCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            DropSpecimenState.class.getSimpleName();
    private int runCounter;

    /**
     * Constructor for the DropState
     *
     * @param name The name of the state
     * @param dropper The dropper subsystem
     */
    public DropSpecimenState(String name, DropperSubsystem dropper, RobotState robotState) {
        super(name);
        runCounter = 0;
        addCommands(
                new DropperFrontSlapAction(dropper, robotState)
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
                return AutoState.SPECIMEN_1_DROP_COMPLETE;
            } else if (runCounter == 2) {
                return AutoState.SPECIMEN_2_DROP_COMPLETE;
            } else if(runCounter == 3) {
                return AutoState.SPECIMEN_3_DROP_COMPLETE;
            } else if (runCounter == 4) {
                return AutoState.SPECIMEN_4_DROP_COMPLETE;
            }
            return AutoState.SPECIMEN_5_DROP_COMPLETE;
        }
        return AutoState.RUNNING;
    }
}
