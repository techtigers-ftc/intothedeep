package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.TransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * Drive state that drives the robot to a sample drop from an intake
 */
public class DriveToGeneralSampleDropState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToGeneralSampleDropState.class.getSimpleName();

    /**
     * Constructor for the DriveToGeneralSampleDropState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param dropper    The dropper subsystem
     * @param intake     The intake subsystem
     * @param robotState The robot state
     */
    public DriveToGeneralSampleDropState(String name, DriveSubsystem drive,
                                         DropperSubsystem dropper,
                                         IntakeSubsystem intake,
                                         double targetSlidePos,
                                         RobotState robotState) {
        super(name, drive, robotState, 3);
        addCommands(
                autoDriveCommand,
                new SequentialCommandGroup(
                        new TransferAction(dropper, intake, robotState),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> robotState.getRobotCurrentPose().getY() < 14),
                                        new IntakeReadyToPickupAction(intake,
                                                robotState, () -> targetSlidePos,
                                                () -> 90)
                                ),
                                new SequentialCommandGroup(
                                        new DropperHighBasketNoTransferAction(dropper, robotState),
                                        new DropperOpenAction(dropper, 100)
                                )
                        )
                )
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.getCurrentCondition() == AutoState.DRIVE_END &&
                robotState.getDropperState() == DropperState.HIGH_BASKET) {
            return AutoState.DRIVE_END;
        } else if (super.getCurrentCondition() == AutoState.TIMEOUT) {
            return AutoState.TIMEOUT;
        }

        return AutoState.RUNNING;
    }
}
