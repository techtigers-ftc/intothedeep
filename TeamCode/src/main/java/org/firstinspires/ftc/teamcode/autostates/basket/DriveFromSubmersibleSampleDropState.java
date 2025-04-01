package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.TransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperHighBasketNoTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.ReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.DropperState;

/**
 * Drive state that drives the robot from the submersible to a sample drop
 */
public class DriveFromSubmersibleSampleDropState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveFromSubmersibleSampleDropState.class.getSimpleName();
    private static final double TIME_TO_INTAKE = 0;
    private boolean isOpenFinished;

    /**
     * Constructor for the DriveFromSubmersibleSampleDropState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public DriveFromSubmersibleSampleDropState(String name, DriveSubsystem drive, DropperSubsystem dropper, IntakeSubsystem intake, RobotState robotState) {
        super(name, drive, robotState, 3);
        addCommands(
                autoDriveCommand,
                new SequentialCommandGroup(
                        new ReadyToTransferAction(intake, dropper, robotState),
//                        new WaitCommand(100),
                        new TransferAction(dropper, intake, robotState),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new DropperHighBasketNoTransferAction(dropper, robotState),
                                        new WaitUntilCommand(() -> robotState.getRobotCurrentPose().getY() < 16),
                                        new DropperOpenAction(dropper, 50),
                                        new InstantCommand(() -> isOpenFinished = true)
                                ),
                                new IntakePrepareToPickupAction(intake, robotState)
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
        if (isOpenFinished &&
                robotState.getDropperState() == DropperState.HIGH_BASKET) {
            if (robotState.getAutoRemainingTime() < TIME_TO_INTAKE) {
                return AutoState.PARK;
            } else {
                return AutoState.DRIVE_END;
            }
        } else if (super.getCurrentCondition() == AutoState.TIMEOUT) {
            return AutoState.TIMEOUT;
        }
        return AutoState.RUNNING;
    }
}
