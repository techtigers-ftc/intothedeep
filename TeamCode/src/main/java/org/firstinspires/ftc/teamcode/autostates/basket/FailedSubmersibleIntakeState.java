package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFinePickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTrackingAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.individualcommands.LimelightBlockDetectionResetAction;
import org.firstinspires.ftc.teamcode.commands.drive.TeleHoldPointAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * State for using the vision system to pick a sample out from the submersible if the first attempt fails
 */
public class FailedSubmersibleIntakeState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG = FailedSubmersibleIntakeState.class.getSimpleName();
    private static final double TIME_TO_INTAKE = 0;
    private static final double TIME_TO_DROP = 0;
    private final RobotState robotState;
    private final IntakeSubsystem intake;
    private int runCounter;
    private String previousAutoState;
    private boolean blockDetected;
    private int frameCounter;

    /**
     * Creates a new FailedSubmersibleIntakeState
     *
     * @param name       the name of the state
     * @param drive      the drive subsystem
     * @param intake     the intake subsystem
     * @param limelight  the limelight subsystem
     * @param robotState the robot state
     */
    public FailedSubmersibleIntakeState(String name, DriveSubsystem drive, IntakeSubsystem intake, LimelightSubsystem limelight, RobotState robotState) {
        super(name, 3);
        this.robotState = robotState;
        this.intake = intake;
        runCounter = 0;
        previousAutoState = "";
        blockDetected = true;
        addCommands(
//                new LimelightBlockDetectionResetAction(limelight),
                new ParallelCommandGroup(
                        new IntakeReadyToPickupAction(intake, robotState, () -> 1.25),
                        new TeleHoldPointAction(
                                drive, robotState,
                                () -> robotState.getRobotCurrentPose().getX(),
                                () -> robotState.getRobotCurrentPose().getY() + 4,
                                () -> robotState.getRobotCurrentPose().getHeading(), 1, Math.toRadians(5)
                        )
                ),
                new WaitUntilCommand(() -> robotState.getRobotVelocity().getPoint().magnitude() < 15),
                new LimelightBlockDetectionResetAction(limelight),
                new IntakeTrackingAction(intake, robotState),
                new WaitUntilCommand(robotState::isBlockDetected),
                new IntakeFinePickupAction(drive, intake, null, robotState)
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        blockDetected = true;
        frameCounter = 0;
    }

    @Override
    public AutoState getCurrentCondition() {
        boolean trackingTimeout = (super.isTimeoutReached() || IntakeSubsystem.SLIDES_MAX - intake.getCurrentSlidePositionInches() < 3.5)
                && !robotState.isVisionAligning();
        boolean blockNotDetected = !blockDetected && !robotState.isVisionAligning() && !robotState.isIntakeTracking();
        if (runCounter == 0) {
            previousAutoState = robotState.getPreviousAutoState();
        }
        if (trackingTimeout || blockNotDetected) {
            runCounter = 0;
            return AutoState.SAMPLE_INTAKE_FAILED;
        } else {
            if (robotState.getAutoRemainingTime() < TIME_TO_INTAKE && robotState.isIntakeTracking()) {
                return AutoState.PARK;
            } else if (robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER && getRunningTime() > 1) {
                if (robotState.getBlockPosition() == RobotBlockPosition.INTAKE || runCounter > 3) {
                    runCounter = 0;
                    if (robotState.getAutoRemainingTime() < TIME_TO_DROP) {
                        return AutoState.PARK;
                    } else {
                        if (previousAutoState.equals("intakeFourthSample")) {
                            return AutoState.SAMPLE_4_INTAKE_RECOVERED;
                        } else {
                            return AutoState.SAMPLE_5_INTAKE_RECOVERED;
                        }
                    }
                } else {
                    if (frameCounter < 5) {
                        frameCounter++;
                        return AutoState.RUNNING;
                    }
                    runCounter++;
                    return AutoState.SAMPLE_INTAKE_FAILED;
                }
            } else {
                return AutoState.RUNNING;
            }
        }
    }
}