package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.TimeoutWaitUntilCommand;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFinePickupAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
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
    private int frameCounter;
    private boolean blockDetected;

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
        blockDetected = true;
        addCommands(
                new WaitUntilCommand(() -> robotState.getRobotVelocity().getPoint().magnitude() < 5),
//                new WaitUntilCommand(robotState::isBlockDetected),
//                new WaitCommand(50),
                new TimeoutWaitUntilCommand(robotState::isBlockDetected, 0.2),
                new InstantCommand(() -> blockDetected = robotState.isBlockDetected()),
                new IntakeFinePickupAction(drive, intake, () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()), robotState)
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        blockDetected = true;
        frameCounter = 0;
    }

    /**
     * Get the current condition of the robot
     *
     * @return the current condition of the robot using the AutoState enum
     */
    @Override
    public AutoState getCurrentCondition() {
        if (super.isTimeoutReached() || !blockDetected) {
            return AutoState.TIMEOUT;
        } else {
            if (robotState.getIntakeState() == IntakeState.PREPARE_TO_TRANSFER) {
                if (robotState.getBlockPosition() == RobotBlockPosition.INTAKE) {
                    return AutoState.SAMPLE_INTAKE_COMPLETE;
                } else {
                    if (frameCounter < 5) {
                        frameCounter++;
                        return AutoState.RUNNING;
                    }
                    return AutoState.SAMPLE_INTAKE_FAILED;
                }
            } else {
                return AutoState.RUNNING;
            }
        }
    }
}
