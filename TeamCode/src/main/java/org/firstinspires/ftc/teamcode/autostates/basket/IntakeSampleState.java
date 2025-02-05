package org.firstinspires.ftc.teamcode.autostates.basket;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFullReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTrackingAction;
import org.firstinspires.ftc.teamcode.subsystems.DropperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;

import java.util.function.DoubleSupplier;

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
     * @param name           The name of the state
     * @param intake         The intake subsystem
     * @param dropper        The dropper subsystem
     * @param targetSlidePos the target position for the slides to move to
     * @param robotState     The robot state
     */
    public IntakeSampleState(String name, IntakeSubsystem intake,
                             DropperSubsystem dropper,
                             DoubleSupplier targetSlidePos,
                             RobotState robotState) {
        super(name, 10);
        this.robotState = robotState;
        addCommands(
//                new IntakePrepareToPickupAction(intake, dropper, () -> targetSlidePos.getAsDouble() - 1, robotState),
                new IntakePrepareToPickupAction(intake, dropper, targetSlidePos, robotState),
                new IntakeTrackingAction(intake, 50, robotState),
                new IntakeReadyToPickupAction(intake, robotState,
                        intake::getCurrentSlidePositionInches,
                        () -> Math.toDegrees(robotState.getRobotCurrentPose().getHeading())),
                new IntakeFullReadyToTransferAction(intake, dropper, robotState, this)
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
