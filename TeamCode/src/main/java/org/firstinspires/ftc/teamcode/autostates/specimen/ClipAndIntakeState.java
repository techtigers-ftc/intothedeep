package org.firstinspires.ftc.teamcode.autostates.specimen;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFinePickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTrackingAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.LimelightBlockDetectionResetAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.drive.RawPowerDriveAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A state to clip a specimen onto the chamber, align to a sample, and pick it up
 */
public class ClipAndIntakeState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            ClipAndIntakeState.class.getSimpleName();
    private final RobotState robotState;
    private final IntakeSubsystem intake;

    /**
     * Constructor for the ClipAndIntakeState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param intake     The intake subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public ClipAndIntakeState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, LimelightSubsystem limelight, RobotState robotState) {
        super(name, 5);
        this.robotState = robotState;
        this.intake = intake;
        addCommands(
                new ParallelCommandGroup(
                        new RawPowerDriveAction(drive, 0.5, 0.1),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(50),
                                        new DropperPitchAction(dropper,
                                                DropperSubsystem.PITCH_SLAP_POSITION, 0),
                                        new WaitCommand(50),
                                        new DropperOpenAction(dropper)
                                ),
                                new SequentialCommandGroup(
                                        new LimelightBlockDetectionResetAction(limelight),
                                        new IntakeTrackingAction(intake, robotState),
                                        new WaitUntilCommand(robotState::isBlockDetected),
                                        new WaitCommand(100)
                                )
                        )),
//                new WaitCommand(100),
//                new WaitUntilCommand(robotState::isBlockDetected),
                new IntakeFinePickupAction(drive, intake, null, robotState)
        );
    }

    /**
     * Get the current condition of the state
     *
     * @return the current condition of the state
     */
    @Override
    public AutoState getCurrentCondition() {
        if (super.isTimeoutReached()) {
            return AutoState.TIMEOUT;
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
