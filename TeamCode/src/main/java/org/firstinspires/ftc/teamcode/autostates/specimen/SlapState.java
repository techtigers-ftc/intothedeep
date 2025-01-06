package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperFrontSlapAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.statemachine.ParallelCommandGroupState;
import team.techtigers.base.statemachine.SequentialCommandGroupState;

public class SlapState extends SequentialCommandGroupState<AutoState> {
    /**
     * Constructor for the ParallelCommandGroupState
     *
     * @param name The name of the state
     */
    public SlapState(String name, DropperSubsystem dropper, RobotState robotState) {
        super(name);
        addCommands(
                new DropperFrontSlapAction(dropper, robotState),
                new DropperOpenAction(dropper, 500)
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (isFinished()) {
            return AutoState.SAMPLE_0_DROP_COMPLETE;
        }
        return AutoState.RUNNING;
    }
}
