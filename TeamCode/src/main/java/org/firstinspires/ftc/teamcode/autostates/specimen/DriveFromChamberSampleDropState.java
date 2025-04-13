package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.ReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.SequentialReadyToTransferAction;
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
public class DriveFromChamberSampleDropState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveFromChamberSampleDropState.class.getSimpleName();
    private final long TIME_TO_WAIT = 0;
    private boolean isOpenFinished;

    /**
     * Constructor for the DriveFromChamberSampleDropState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param intake     The intake subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public DriveFromChamberSampleDropState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState);
        addCommands(
                new SequentialCommandGroup(
                        new WaitCommand(TIME_TO_WAIT),
                        autoDriveCommand

                        ),
                new SequentialCommandGroup(
                        new SequentialReadyToTransferAction(intake, dropper, robotState),
                        new TransferAction(dropper, intake, robotState),
                        new ParallelCommandGroup(
                                new DropperHighBasketNoTransferAction(dropper, robotState),
                                new SequentialCommandGroup(
                                        new WaitUntilCommand(() -> (robotState.getAutoRemainingTime() < 0.1 || robotState.getRobotCurrentPose().getX() < 14) && dropper.getPitch() > 190),
                                        new DropperOpenAction(dropper, 150),
                                        new InstantCommand(() -> isOpenFinished = true)
                                ),
                                new IntakeTuckAfterTransferAction(dropper, intake, robotState)
                        )
                )
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        isOpenFinished = false;
    }

    @Override
    public AutoState getCurrentCondition() {
        if (isOpenFinished) {
            return AutoState.DRIVE_END;
        } else if (super.getCurrentCondition() == AutoState.TIMEOUT) {
            return AutoState.TIMEOUT;
        }
        return AutoState.RUNNING;
    }
}
