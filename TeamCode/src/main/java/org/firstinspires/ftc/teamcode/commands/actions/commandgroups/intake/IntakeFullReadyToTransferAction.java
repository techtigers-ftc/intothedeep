package org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.TeleHoldPointAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeClawRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;

/**
 * Command to use vision to align the robot to a block and hover over it
 */
public class IntakeFullReadyToTransferAction extends SequentialCommandGroup {
    private static final String LOG_TAG = IntakePrepareToPickupAction.class.getSimpleName();
    private final RobotState robotState;
    private final IntakeSubsystem intake;

    /**
     * Creates a new IntakeFullReadyToTransferAction
     *
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     * @param command    the command to cancel
     */
    public IntakeFullReadyToTransferAction(DriveSubsystem drive,
                                           IntakeSubsystem intake,
                                           DropperSubsystem dropper,
                                           RobotState robotState, CommandBase command) {
        this.robotState = robotState;
        this.intake = intake;
        addRequirements(intake, dropper);

        TeleHoldPointAction holdPointAction =
                new TeleHoldPointAction(drive, robotState,
                        () -> robotState.getRobotCurrentPose().getX(),
                        () -> robotState.getRobotCurrentPose().getY(),
                        () -> robotState.getRobotCurrentPose().getHeading(),
                        0, Math.toRadians(0)
                );
        addCommands(
                new InstantCommand(() -> robotState.setVisionAligning(true)),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        new IntakeSlidesAbsoluteAction(intake,
                                () -> intake.getCurrentSlidePositionInches() + robotState.getBlockForwardFine() + 3, 0.75),
                        new IntakeClawRotationAction(intake, robotState::getBlockOrientation, 300),
                        new TeleHoldPointAction(drive, robotState,
                                () -> robotState.getRobotCurrentPose().getX() +
                                        Math.sin(robotState.getRobotCurrentPose().getHeading()) * (robotState.getBlockLateralFine()),
                                () -> robotState.getRobotCurrentPose().getY()
                                        - Math.cos(robotState.getRobotCurrentPose().getHeading()) * (robotState.getBlockLateralFine()),
                                () -> robotState.getRobotCurrentPose().getHeading(), 0.5, Math.toRadians(2))
                ),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                holdPointAction,
                                new WaitUntilCommand(robotState::isVisionAligning),
                                new InstantCommand(holdPointAction::stop)
                        ),
                        new IntakeFullReadyToTransferNoVisionAction(intake,
                                dropper, robotState, command == null ? this : command)
                )
        );
    }

    /**
     * Overload constructor for if the command doesn't receive a command to cancel
     *
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public IntakeFullReadyToTransferAction(DriveSubsystem drive,
                                           IntakeSubsystem intake,
                                           DropperSubsystem dropper,
                                           RobotState robotState) {
        this(drive, intake, dropper, robotState, null);
    }
}
