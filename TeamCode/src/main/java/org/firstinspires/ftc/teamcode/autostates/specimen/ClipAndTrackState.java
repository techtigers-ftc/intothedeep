package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTrackingAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
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
 * A state to clip a specimen onto the chamber, track a sample, and pick it up
 */
public class ClipAndTrackState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            ClipAndTrackState.class.getSimpleName();
    private static final double TIME_TO_DROP = 2.5;
    private RobotState robotState;
    private IntakeSubsystem intake;

    /**
     * Constructor for the ClipAndTrackState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param intake     The intake subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public ClipAndTrackState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name, 2.5);
        this.robotState = robotState;
        this.intake = intake;
        addCommands(
                new ParallelCommandGroup(
                        new RawPowerDriveAction(drive, 0.8, 0.25),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new DropperPitchAction(dropper,
                                        DropperSubsystem.PITCH_FRONT_SLAP_POSITION, 0),
                                new WaitCommand(150),
                                new DropperOpenAction(dropper)
                        ),
                        new IntakeTrackingAction(intake, robotState)
                ),
                new IntakePrepareToTransferAction(drive, intake, robotState::getBlockOrientation, robotState)
        );
    }

    /**
     * Get the current condition of the state
     *
     * @return the current condition of the state
     */
    @Override
    public AutoState getCurrentCondition() {
        if (super.isTimeoutReached() || (IntakeSubsystem.SLIDES_MAX - intake.getCurrentSlidePositionInches() < 2.5 && robotState.isIntakeTracking())) {
            return AutoState.TIMEOUT;
        } else if (robotState.getAutoRemainingTime() < TIME_TO_DROP) {
            return AutoState.NO_TIME;
        } else {
            if (robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER) {
                if (robotState.getBlockPosition() == RobotBlockPosition.INTAKE) {
                    return AutoState.SAMPLE_INTAKE_COMPLETE;
                } else {
                    return AutoState.SAMPLE_INTAKE_FAILED;
                }
            } else {
                return AutoState.RUNNING;
            }
        }
    }
}
