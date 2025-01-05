package org.firstinspires.ftc.teamcode.autostates;

import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.statemachine.ParallelCommandGroupState;

/**
 * A state to drop a block
 */
public class FirstLevelAscent extends ParallelCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            FirstLevelAscent.class.getSimpleName();
    private int runCounter;

    /**
     * Constructor for the FirstLevelAscent
     *
     * @param name The name of the state
     */
    public FirstLevelAscent(String name, DropperSubsystem dropper) {
        super(name);
        addCommands(
                new DropperPitchAction(dropper, DropperSubsystem.PITCH_FIRST_LEVEL_ASCENT, 500)
        );
    }

    /**
     * Get the current condition of the robot
     * @return the current condition of the robot using the AutoState enum
     */
    @Override
    public AutoState getCurrentCondition() {
        return AutoState.RUNNING;
    }
}
