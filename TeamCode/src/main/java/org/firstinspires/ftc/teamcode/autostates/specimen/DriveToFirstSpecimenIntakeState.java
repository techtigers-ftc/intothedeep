package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.autostates.DriveStateBase;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.ReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.TransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeTuckAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperSlidesAbsoluteAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.intake.IntakeWristPitchAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.ClawState;

/**
 * Drive state that drives the robot from the chamber to a sample drop
 */
public class DriveToFirstSpecimenIntakeState extends DriveStateBase {
    private static final String LOG_TAG =
            DriveToFirstSpecimenIntakeState.class.getSimpleName();

    /**
     * Constructor for the DriveFromWallSampleDropState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public DriveToFirstSpecimenIntakeState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name, drive, robotState);
        addCommands(
                autoDriveCommand,
//                new IntakeTuckAction(intake, robotState),
                new SequentialCommandGroup(
                        new ReadyToTransferAction(intake, dropper, robotState),
//                        new ParallelReadyToTransferAction(intake, dropper, robotState),
                        new TransferAction(dropper, intake, robotState),
                        new IntakeWristPitchAction(intake,
                                IntakeSubsystem.WRIST_PITCH_TRANSFER_POSITION + 20, 15),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(100),
                                        new IntakeTuckAction(intake, robotState)
                                ),
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new DropperSlidesAbsoluteAction(dropper, DropperSubsystem.SLIDES_WALL_INTAKE_POSITION, 0.75),
                                                new DropperPitchAction(dropper, DropperSubsystem.PITCH_WALL_INTAKE_POSITION, 300),
                                                new DropperRotationAction(dropper, DropperSubsystem.ROTATION_WALL_INTAKE_POSITION, 0)
                                        ),
                                        new DropperOpenAction(dropper)
                                )
                        )
//                        new WaitUntilCommand(() -> robotState.getRobotCurrentPose().getY() < 30),
                )
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (super.getCurrentCondition() == AutoState.DRIVE_END &&
                robotState.getDropperClawState() == ClawState.OPEN) {
            return AutoState.DRIVE_END;
        } else if (super.getCurrentCondition() == AutoState.TIMEOUT) {
            return AutoState.TIMEOUT;
        }
        return AutoState.RUNNING;
    }
}
