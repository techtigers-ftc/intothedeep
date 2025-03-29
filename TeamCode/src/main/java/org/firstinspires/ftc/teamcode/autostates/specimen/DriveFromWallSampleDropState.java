package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.ReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.TransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTuckAfterTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * Drive state that drives the robot from the chamber to a sample drop
 */
public class DriveFromWallSampleDropState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveFromWallSampleDropState.class.getSimpleName();

    /**
     * Constructor for the DriveFromWallSampleDropState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param intake     The intake subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public DriveFromWallSampleDropState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState);
        addCommands(
                autoDriveCommand,
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> robotState.getRobotCurrentPose().getX() < 80),
                        new DropperHighBasketNoTransferAction(dropper, robotState),
                        new WaitUntilCommand(() -> robotState.getRobotCurrentPose().getX() < 14),
                        new DropperOpenAction(dropper, 50)
                )
//
//                new SequentialCommandGroup(
//                        new ReadyToTransferAction(intake, dropper, robotState),
//                        new TransferAction(dropper, intake, robotState),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new DropperHighBasketNoTransferAction(dropper, robotState),
//                                        new DropperOpenAction(dropper, 100)
//                                ),
//                                new SequentialCommandGroup(
//                                        new WaitUntilCommand(() -> robotState.getAutoRemainingTime() < 0.1 && dropper.getPitch() > 180),
//                                        new DropperOpenAction(dropper)
//                                ),
//                                new IntakeTuckAfterTransferAction(dropper, intake, robotState)
//                        )
//                )
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.getCurrentCondition() == AutoState.DRIVE_END &&
                robotState.getDropperClawState() == ClawState.OPEN &&
                robotState.getDropperState() == DropperState.HIGH_BASKET) {
            return AutoState.DRIVE_END;
        } else if (super.getCurrentCondition() == AutoState.TIMEOUT) {
            return AutoState.TIMEOUT;
        }
        return AutoState.RUNNING;
    }
}
