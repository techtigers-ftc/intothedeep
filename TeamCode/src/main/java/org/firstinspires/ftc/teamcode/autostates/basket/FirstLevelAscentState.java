package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A State to move the dropper to the first level ascent
 */
public class FirstLevelAscentState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            FirstLevelAscentState.class.getSimpleName();
    private final RobotState robotState;
    private final DropperSubsystem dropper;

    /**
     * Constructor for the FirstLevelAscentState
     *
     * @param name       The name of the state
     * @param dropper    The dropper subsystem
     * @param intake     The intake subsystem
     * @param robotState The robot state
     */
    public FirstLevelAscentState(String name, DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        super(name, 5);
        this.robotState = robotState;
        this.dropper = dropper;
        addCommands(
                new ParallelCommandGroup(
                        new DropperPitchAction(dropper, DropperSubsystem.PITCH_PRE_TRANSFER_POSITION,
                                300),
                        new DropperRotationAction(dropper, DropperSubsystem.ROTATION_TRANSFER_POSITION, 300),
                        new IntakeTuckAction(intake, robotState),
                        new DropperSlidesAbsoluteAction(dropper, 17, 1)
                )
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (dropper.getCurrentSlidePositionInches() > 16.75) {
            return AutoState.ASCENT_COMPLETE;
        } else if (isTimeoutReached()) {
            return AutoState.TIMEOUT;
        }
        return AutoState.RUNNING;
    }
}
