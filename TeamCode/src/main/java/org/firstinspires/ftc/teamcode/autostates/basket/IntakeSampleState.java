package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.VisionIntakeBlockAutonomous;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.BlockDetectionState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

import team.techtigers.base.statemachine.SequentialCommandGroupState;

/**
 * A state to intake a sample for the basket auto
 */
public class IntakeSampleState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            IntakeSampleState.class.getSimpleName();
    private final RobotState robotState;

    /**
     * Constructor for the IntakeSampleState
     *
     * @param name       The name of the state
     * @param drive      The drive subsystem
     * @param intake     The intake subsystem
     * @param dropper    The dropper subsystem
     * @param robotState The robot state
     */
    public IntakeSampleState(String name, DriveSubsystem drive, IntakeSubsystem intake,
                             DropperSubsystem dropper,
                             RobotState robotState) {
        super(name, 3);
        this.robotState = robotState;
        addCommands(
                // TODO: Tune this wait time
                new WaitUntilCommand(() -> robotState.getFineBlockDetectionState() == BlockDetectionState.DETECTED),
                new VisionIntakeBlockAutonomous(drive, intake, dropper, () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading()), robotState)
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
            if (robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER) {
                return AutoState.SAMPLE_INTAKE_COMPLETE;
            } else {
                return AutoState.RUNNING;
            }
        }
    }
}
