package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeCoarseAlignAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFinePickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.drive.TeleHoldPointAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * State for using the vision system to pick a sample out from the submersible if the first attempt fails
 */
public class FailedSubmersibleIntakeState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG = FailedSubmersibleIntakeState.class.getSimpleName();
    private static final double TIME_TO_INTAKE = 2;
    private static final double TIME_TO_DROP = 2.5;
    private final RobotState robotState;
    private int runCounter;
    private String previousAutoState;
    private boolean blockDetected;

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
        runCounter = 0;
        previousAutoState = "";
        blockDetected = true;
        addCommands(
                new ParallelCommandGroup(
                        new IntakePrepareToPickupAction(intake, robotState),
                        new TeleHoldPointAction(
                                drive, robotState,
                                () -> robotState.getRobotCurrentPose().getX(),
                                () -> robotState.getRobotCurrentPose().getY() + 4,
                                () -> robotState.getRobotCurrentPose().getHeading(), 1, Math.toRadians(5)
                        )
                ),
                new WaitUntilCommand(() -> robotState.getRobotVelocity().getPoint().magnitude() < 2),
                new IntakeCoarseAlignAction(drive, intake, robotState),
                new WaitCommand(100),
                new InstantCommand(() -> blockDetected = robotState.getFineBlockDetectionState() == BlockDetectionState.DETECTED),
                new IntakeFinePickupAction(drive, intake, robotState)
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        blockDetected = true;
    }

    @Override
    public AutoState getCurrentCondition() {
        boolean blockNotDetected = !blockDetected && !robotState.isVisionAligning() && !robotState.isIntakeTracking();
        if (runCounter == 0) {
            previousAutoState = robotState.getPreviousAutoState();
        }
        if (blockNotDetected || super.isTimeoutReached()) {
            runCounter = 0;
            if (previousAutoState.equals("intakeFourthSample")) {
                return AutoState.FAILED_SAMPLE_4_TIMEOUT;
            } else {
                return AutoState.FAILED_SAMPLE_5_TIMEOUT;
            }
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
                    runCounter++;
                    return AutoState.SAMPLE_INTAKE_FAILED;
                }
            } else {
                return AutoState.RUNNING;
            }
        }
    }
}
