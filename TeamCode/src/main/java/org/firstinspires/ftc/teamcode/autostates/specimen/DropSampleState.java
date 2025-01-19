package org.firstinspires.ftc.teamcode.autostates.specimen;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperBackSlapAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperBackwardCarryAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToTransferAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A state to grab a sample to be dropped off later
 */
public class DropSampleState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            DropSampleState.class.getSimpleName();
    private final RobotState robotState;


    /**
     * Constructor for the GrabSampleState
     *
     * @param name       The name of the state
     * @param intake     The intake subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public DropSampleState(String name, IntakeSubsystem intake, DropperSubsystem dropper,
                           RobotState robotState) {
        super(name);
        this.robotState = robotState;
        addCommands(
                new IntakeReadyToTransferAction(intake, robotState),
                new DropperBackwardCarryAction(dropper, intake, robotState),
                new DropperBackSlapAction(dropper, robotState)
        );
    }

    /**
     * Get the current condition of the robot
     *
     * @return the current condition of the robot using the AutoState enum
     */
    @Override
    public AutoState getCurrentCondition() {
        if (robotState.getDropperState() == DropperState.BACK_SLAP) {
            return AutoState.SAMPLE_0_DROP_COMPLETE;
        }
        return AutoState.RUNNING;
    }
}
