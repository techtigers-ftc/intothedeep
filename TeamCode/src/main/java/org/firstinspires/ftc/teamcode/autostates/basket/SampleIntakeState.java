package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A state to intake a sample for the basket auto
 */
public class SampleIntakeState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            SampleIntakeState.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Constructor for the SampleIntakeState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param intake     The intake subsystem
     * @param robotState The robot state
     */
    public SampleIntakeState(String name, DriveSubsystem drive, IntakeSubsystem intake,
                             RobotState robotState) {
        super(name, 3);
        this.robotState = robotState;
        addCommands(
                new WaitUntilCommand(() -> robotState.getRobotVelocity().getPoint().magnitude() < 2),
                new WaitUntilCommand(() -> robotState.getFineBlockDetectionState() == BlockDetectionState.DETECTED),
                new IntakePrepareToTransferAction(drive, intake, () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()), robotState)
        );
    }

    /**
     * Get the current condition of the robot
     *
     * @return the current condition of the robot using the AutoState enum
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
