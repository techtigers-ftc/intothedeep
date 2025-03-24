package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFinePickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.states.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A state to intake one of the three samples off the floor using vision in auto if the first intake fails
 */
public class FailedIntakeSampleState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            FailedIntakeSampleState.class.getSimpleName();
    private final RobotState robotState;
    private int runCounter;
    private String previousAutoState;

    /**
     * Constructor for the FailedIntakeSampleState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param intake     The intake subsystem
     * @param robotState The robot state
     */
    public FailedIntakeSampleState(String name, DriveSubsystem drive, IntakeSubsystem intake,
                                   RobotState robotState) {
        super(name, 3);
        this.robotState = robotState;
        runCounter = 0;
        previousAutoState = "";
        addCommands(
                new IntakeReadyToPickupAction(intake, robotState, () -> intake.getCurrentSlidePositionInches() - 3.5),
                new WaitUntilCommand(() -> robotState.getFineBlockDetectionState() == BlockDetectionState.DETECTED),
                new IntakeFinePickupAction(drive, intake, robotState)
        );
    }

    @Override
    public AutoState getCurrentCondition() {
        if (runCounter == 0) {
            previousAutoState = robotState.getPreviousAutoState();
        }
        if (super.isTimeoutReached()) {
            runCounter = 0;
            if (previousAutoState.equals("intakeFirstSample")) {
                return AutoState.FAILED_SAMPLE_1_TIMEOUT;
            } else if (previousAutoState.equals("intakeSecondSample")) {
                return AutoState.FAILED_SAMPLE_2_TIMEOUT;
            } else {
                return AutoState.FAILED_SAMPLE_3_TIMEOUT;
            }
        } else {
            if (robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER && getRunningTime() > 1) {
                if (robotState.getBlockPosition() == RobotBlockPosition.INTAKE || runCounter > 0) {
                    runCounter = 0;
                    if (previousAutoState.equals("intakeFirstSample")) {
                        return AutoState.SAMPLE_1_INTAKE_RECOVERED;
                    } else if (previousAutoState.equals("intakeSecondSample")) {
                        return AutoState.SAMPLE_2_INTAKE_RECOVERED;
                    } else {
                        return AutoState.SAMPLE_3_INTAKE_RECOVERED;
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
