package org.firstinspires.ftc.teamcode.autostates;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeReadyToTransferAction;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotState;
import org.firstinspires.ftc.teamcode.utils.enums.AutoState;
import org.firstinspires.ftc.teamcode.utils.enums.IntakeState;
import org.firstinspires.ftc.teamcode.utils.enums.RobotBlockPosition;

import java.util.function.DoubleSupplier;

import team.techtigers.base.statemachine.SequentialCommandGroupState;
import team.techtigers.core.paths.Waypoint;

/**
 * A state to intake a sample
 */
public class IntakeSampleState extends SequentialCommandGroupState<AutoState> {
    private static final String LOG_TAG =
            IntakeSampleState.class.getSimpleName();
    private final RobotState robotState;
    private DoubleSupplier slidePos;
    private DoubleSupplier clawPos;

    /**
     * Constructor for the IntakeSampleState
     *
     * @param name The name of the state
     * @param intake The intake subsystem
     */
    public IntakeSampleState(String name, IntakeSubsystem intake,
                             RobotState robotState, DoubleSupplier targetSlidePos,
                             DoubleSupplier targetClawRotation) {
        super(name);
        this.robotState = robotState;
        slidePos = targetSlidePos;
        clawPos = targetClawRotation;
        addCommands(
                new IntakePrepareToPickupAction(intake, robotState, targetSlidePos),
                new IntakeReadyToPickupAction(intake, robotState,
                        targetSlidePos, targetClawRotation),
                new IntakePrepareToTransferAction(intake, robotState),
                new IntakeReadyToTransferAction(intake, robotState)
        );
    }

    /**
     * Get the current condition of the robot
     */
    @Override
    public void initialize() {
        super.initialize();
        RobotLog.dd(LOG_TAG, "Target slide pos: %s Target Claw pos %s",
                slidePos.getAsDouble(), clawPos.getAsDouble());
    }

    /**
     * Get the current condition of the robot
     * @return the current condition of the robot using the AutoState enum
     */
    @Override
    public AutoState getCurrentCondition() {
        if (robotState.getIntakeState() == IntakeState.READY_TO_TRANSFER &&
            robotState.getBlockPosition() == RobotBlockPosition.INTAKE) {
            return AutoState.SAMPLE_INTAKE_COMPLETE;
        }
        return AutoState.RUNNING;
    }
}
