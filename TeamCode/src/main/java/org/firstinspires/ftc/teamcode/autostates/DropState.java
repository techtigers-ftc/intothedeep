package org.firstinspires.ftc.teamcode.autostates;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.FirstDriveToBasketCommand;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

import team.techtigers.base.statemachine.ParallelCommandGroupState;
import team.techtigers.core.paths.Waypoint;

/**
 * A state to drop a block
 */
public class DropState extends ParallelCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            DropState.class.getSimpleName();
    private RobotState robotState;

    /**
     * Constructor for the SequentialCommandGroupState
     *
     * @param name The name of the state
     */
    public DropState(String name, DropperSubsystem dropper, RobotState robotState) {
        super(name);
        this.robotState = robotState;
        addCommands(
                new DropperOpenAction(dropper, 200)
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        return isFinished()? AutoState.END_1: AutoState.RUNNING;
    }
}
