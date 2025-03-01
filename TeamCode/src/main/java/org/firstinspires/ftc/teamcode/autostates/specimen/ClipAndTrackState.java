package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTrackingAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.drive.RawPowerDriveAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A state to clip a specimen onto the chamber
 */
public class ClipAndTrackState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            ClipAndTrackState.class.getSimpleName();
    private RobotState robotState;
    private IntakeSubsystem intake;

    /**
     * Constructor for the ClipSpecimenState
     *
     * @param name       The name of the state
     * @param dropper    The dropper subsystem
     * @param drive      The drive subsystem
     * @param robotState The robot state
     */
    public ClipAndTrackState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name, 10);
        this.robotState = robotState;
        this.intake = intake;
        addCommands(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new RawPowerDriveAction(drive, 0.4, 0.1),
                                new DropperPitchAction(dropper,
                                        DropperSubsystem.PITCH_FRONT_SLAP_POSITION, 0),
                                new WaitCommand(300)
                        ),
                        new SequentialCommandGroup(
                                new IntakeReadyToPickupAction(intake, robotState, () -> 0, () -> 90),
                                new IntakeTrackingAction(intake, robotState))
                ),
                new IntakePrepareToTransferAction(drive, intake, dropper, robotState::getBlockOrientation, robotState)
        );
    }

    /**
     * Get the current condition of the state
     *
     * @return the current condition of the state based on run counter
     */
    @Override
    public AutoState getCurrentCondition() {
        if (super.isTimeoutReached() || (IntakeSubsystem.SLIDES_MAX - intake.getCurrentSlidePositionInches() < 2.5 && robotState.isIntakeTracking())) {
            return AutoState.TIMEOUT;
        } else {
            if (robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER) {
                    return AutoState.SAMPLE_INTAKE_COMPLETE;
            } else {
                return AutoState.RUNNING;
            }
        }
    }
}
