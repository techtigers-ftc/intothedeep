package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.IntakeTuckAfterTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.TransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.SubmersibleIntakeReadyToTransferAction;
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
public class FirstLevelAscentTransferState extends ParallelCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            FirstLevelAscentTransferState.class.getSimpleName();

    /**
     * Constructor for the FirstLevelAscentState
     *
     * @param name       The name of the state
     * @param drive      the drive subsystem
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public FirstLevelAscentTransferState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name);
        addCommands(
                new RawPowerDriveAction(drive, 1, 10),
                new SequentialCommandGroup(
                        new SubmersibleIntakeReadyToTransferAction(intake, dropper, robotState),
                        new WaitCommand(1000),
                        new TransferAction(dropper, intake, robotState),
                        new ParallelCommandGroup(
                                new IntakeTuckAfterTransferAction(dropper, intake, robotState),
                                new DropperSlidesAbsoluteAction(dropper, 17, 1)
                        )
                )
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        return AutoState.RUNNING;
    }
}
