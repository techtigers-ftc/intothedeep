package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.TeleHoldPointAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTrackingAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
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
    private static final double TIME_TO_INTAKE = 1;
    private static final double TIME_TO_DROP = 3;
    private final RobotState robotState;
    private final IntakeSubsystem intake;
    private int runCounter;
    private String previousAutoState;

    /**
     * Creates a new FailedSubmersibleIntakeState
     *
     * @param name       the name of the state
     * @param drive      the drive subsystem
     * @param intake     the intake subsystem
     * @param dropper    the dropper subsystem
     * @param robotState the robot state
     */
    public FailedSubmersibleIntakeState(String name, DriveSubsystem drive, IntakeSubsystem intake, DropperSubsystem dropper, RobotState robotState) {
        super(name, 3);
        this.robotState = robotState;
        this.intake = intake;
        runCounter = 0;
        previousAutoState = "";
        addCommands(
                new ParallelCommandGroup(
                        new IntakeReadyToPickupAction(intake, robotState, () -> 1.75, () -> 90),
                        new TeleHoldPointAction(
                                drive, robotState,
                                () -> robotState.getRobotCurrentPose().getX(),
                                () -> robotState.getRobotCurrentPose().getY() + 4,
                                () -> robotState.getRobotCurrentPose().getHeading(), 1, Math.toRadians(5)
                        )
                ),
                new IntakeTrackingAction(intake, robotState),
                new IntakePrepareToTransferAction(drive, intake, robotState::getBlockOrientation, robotState)
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (runCounter == 0) {
            previousAutoState = robotState.getPreviousAutoState();
        }
        if (super.isTimeoutReached() || (IntakeSubsystem.SLIDES_MAX - intake.getCurrentSlidePositionInches() < 2.5 && robotState.isIntakeTracking())) {
            runCounter = 0;
            if (previousAutoState.equals("intakeFourthSample")) {
                return AutoState.FAILED_SAMPLE_4_TIMEOUT;
            } else {
                return AutoState.FAILED_SAMPLE_5_TIMEOUT;
            }
        } else {
            if (robotState.getAutoRemainingTime() < TIME_TO_INTAKE && robotState.isIntakeTracking()) {
                RobotLog.dd(LOG_TAG, "Going to park");
                return AutoState.PARK;
            } else if (robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER && getRunningTime() > 1) {
                if (robotState.getBlockPosition() == RobotBlockPosition.INTAKE || runCounter > 3) {
                    runCounter = 0;
                    if (robotState.getAutoRemainingTime() < TIME_TO_DROP) {
                        RobotLog.dd(LOG_TAG, "no time");
                        return AutoState.NO_TIME;
                    } else {
                        if (previousAutoState.equals("intakeFourthSample")) {
                            return AutoState.SAMPLE_4_INTAKE_RECOVERED;
                        } else {
                            return AutoState.SAMPLE_5_INTAKE_RECOVERED;
                        }
                    }
                } else {
                    runCounter++;
                    return AutoState.SAMPLE_INTAKE_FAILED;
                }
            } else {
                return AutoState.RUNNING;
            }
        }
    }
}
