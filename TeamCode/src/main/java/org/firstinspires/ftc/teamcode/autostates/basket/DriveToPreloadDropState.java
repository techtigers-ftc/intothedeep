package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

/**
 * Drives to Preload Drop
 */
public class DriveToPreloadDropState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToPreloadDropState.class.getSimpleName();
    private boolean isOpenFinished;

    /**
     * Constructor for the DriveToPreloadDropState
     *
     * @param name           The name of the state
     * @param drive          The drive subsystem
     * @param dropper        The dropper subsystem
     * @param intake         The intake subsystem
     * @param targetSlidePos The target position of the intake slides
     * @param robotState     The robot state
     */
    public DriveToPreloadDropState(String name, DriveSubsystem drive,
                                   DropperSubsystem dropper,
                                   IntakeSubsystem intake,
                                   double targetSlidePos,
                                   RobotState robotState) {
        super(name, drive, robotState, 4);
        addCommands(
                autoDriveCommand,
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> robotState.getRobotCurrentPose().getX() < 25),
                                new IntakeReadyToPickupAction(intake, robotState, () -> targetSlidePos)
                        ),
                        new SequentialCommandGroup(
                                new DropperHighBasketNoTransferAction(dropper, robotState),
                                new WaitUntilCommand(() -> robotState.getRobotCurrentPose().getX() < 15),
                                new DropperOpenAction(dropper, 100),
                                new InstantCommand(() -> isOpenFinished = true)
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
        if (robotState.getDropperState() == DropperState.HIGH_BASKET &&
                isOpenFinished &&
                robotState.getIntakeState() == IntakeState.READY_TO_PICKUP) {
            return AutoState.DRIVE_END;
        } else if (super.getCurrentCondition() == AutoState.TIMEOUT) {
            return AutoState.TIMEOUT;
        }

        return AutoState.RUNNING;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robotState.getAbsoluteBlockPosition().resetBlockDetection();
    }
}
