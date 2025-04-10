package org.firstinspires.ftc.teamcode.autostates.visiontest;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFinePickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTrackingAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.LimelightBlockDetectionResetAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.dropper.DropperOpenAction;
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
 * State for running the tracking and picking up a sample with the fine vision
 */
public class TrackAndIntakeState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG = TrackAndIntakeState.class.getSimpleName();
    private static final double TIME_TO_INTAKE = 0;
    private static final double TIME_TO_DROP = 0;
    private final RobotState robotState;
    private final IntakeSubsystem intake;
    private boolean blockDetected;
    private double frameCount;

    /**
     * Creates a new TrackAndIntakeState
     *
     * @param name       the name of the state
     * @param drive      the drive subsystem
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param limelight  the limelight subsystem
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
        frameCount = 0;
    }

    @Override
    public AutoState getCurrentCondition() {
        boolean trackingTimeout = super.isTimeoutReached() || (IntakeSubsystem.SLIDES_MAX - intake.getCurrentSlidePositionInches() < 1.5
                && robotState.isIntakeTracking());
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
                    if (frameCount < 5) {
                        frameCount++;
                        return AutoState.RUNNING;
                    }
                    RobotLog.dd("AutoVisionDebug", "Sample Intake Failed");
                    return AutoState.SAMPLE_INTAKE_FAILED;
                }
            } else {
                return AutoState.RUNNING;
            }
        }
    }
}