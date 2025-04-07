package org.firstinspires.ftc.teamcode.autostates.visiontest;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.dropper.DropperPreTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFinePickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTrackingAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.LimelightBlockDetectionResetAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperPitchAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperRotationAction;
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
 * State for using the vision system to pick a sample out from the submersible
 */
public class TrackAndIntakeState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG = TrackAndIntakeState.class.getSimpleName();
    private static final double TIME_TO_INTAKE = 0;
    private static final double TIME_TO_DROP = 0;
    private final RobotState robotState;
    private final IntakeSubsystem intake;
    private boolean blockDetected;

    /**
     * Creates a new SubmersibleIntakeState
     *
     * @param name       the name of the state
     * @param drive      the drive subsystem
     * @param intake     the intake subsystem
     * @param robotState the robot state
     */
    public TrackAndIntakeState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, LimelightSubsystem limelight, RobotState robotState) {
        super(name, 5);
        this.robotState = robotState;
        this.intake = intake;
        blockDetected = true;
        addCommands(
                new IntakeReadyToPickupAction(intake, robotState, () -> 0),
                new LimelightBlockDetectionResetAction(limelight),
                new IntakeTrackingAction(intake, robotState),
                new WaitUntilCommand(robotState::isBlockDetected),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new IntakeFinePickupAction(drive, intake, null, robotState)
                        ),
                        new SequentialCommandGroup(
                            new DropperOpenAction(dropper),
                            new DropperRotationAction(dropper, DropperSubsystem.ROTATION_TRANSFER_POSITION, 100)
                        )
                )
                );
    }

    @Override
    public void initialize() {
        super.initialize();
        robotState.setIntakeState(IntakeState.READY_TO_PICKUP);
        blockDetected = true;
    }

    @Override
    public AutoState getCurrentCondition() {
        boolean trackingTimeout = super.isTimeoutReached() || (IntakeSubsystem.SLIDES_MAX - intake.getCurrentSlidePositionInches() < 1.5
                && !robotState.isVisionAligning());
        boolean blockNotDetected = !blockDetected && !robotState.isVisionAligning() && !robotState.isIntakeTracking();
        if (trackingTimeout || blockNotDetected) {
            RobotLog.dd("AutoVisionDebug", "Timeout");
            return AutoState.TIMEOUT;
        } else {
            if (robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER) {
                RobotLog.dd("AutoVisionDebug", "Intake in Prepare To Transfer");
                if (robotState.getBlockPosition() == RobotBlockPosition.INTAKE) {
                    RobotLog.dd("AutoVisionDebug", "Sample Intake Complete");
                    return AutoState.SAMPLE_INTAKE_COMPLETE;
                } else {
                    RobotLog.dd("AutoVisionDebug", "Sample Intake Failed");
                    return AutoState.SAMPLE_INTAKE_FAILED;
                }
            } else {
                return AutoState.RUNNING;
            }
        }
    }
}