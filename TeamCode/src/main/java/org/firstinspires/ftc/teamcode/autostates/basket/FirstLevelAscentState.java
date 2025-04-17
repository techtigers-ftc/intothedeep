package org.firstinspires.ftc.teamcode.autostates.basket;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.drive.RawPowerDriveAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;

import team.techtigers.base.statemachine.ParallelCommandGroupState;

/**
 * A state to bring up the slides so the wire guide touches the first bar for a level 1 ascent
 */
public class FirstLevelAscentState extends ParallelCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            FirstLevelAscentState.class.getSimpleName();

    /**
     * Constructor for the FirstLevelAscentState
     *
     * @param name       The name of the state
     * @param drive      the drive subsystem
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public FirstLevelAscentState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name);
        addCommands(
                new DropperPitchAction(dropper, DropperSubsystem.PITCH_INIT_POSITION,
                        300),
                new DropperRotationAction(dropper, DropperSubsystem.ROTATION_TRANSFER_POSITION, 300),
                new IntakeTuckAction(intake, robotState),
                new DropperSlidesAbsoluteAction(dropper, 17, 1),
                new RawPowerDriveAction(drive, 1, 1)
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        return AutoState.RUNNING;
    }
}
