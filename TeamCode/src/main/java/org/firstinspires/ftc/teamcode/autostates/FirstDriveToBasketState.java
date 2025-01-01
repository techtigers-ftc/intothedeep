package org.firstinspires.ftc.teamcode.autostates;

import org.firstinspires.ftc.teamcode.commands.FirstDriveToBasketCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A test autonomous state that drives the robot using PedroPathing.
 */
public class FirstDriveToBasketState extends SequentialCommandGroupState<AutoState> {
    /**
     * Constructor for the SequentialCommandGroupState
     *
     * @param name The name of the state
     */
    public FirstDriveToBasketState(String name, DriveSubsystem drive, RobotState robotState) {
        super(name);
        addCommands(
                new FirstDriveToBasketCommand(drive, robotState)
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        return isFinished()? AutoState.END_1 : AutoState.RUNNING;
    }
}
