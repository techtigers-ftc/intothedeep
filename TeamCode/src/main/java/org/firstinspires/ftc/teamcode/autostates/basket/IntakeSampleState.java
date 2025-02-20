package org.firstinspires.ftc.teamcode.autostates.basket;

import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFullReadyToTransferAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeFullReadyToTransferNoVisionAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.SmallCameraVisionPickup;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakePrepareToPickupAction;
import org.firstinspires.ftc.teamcode.commands.actions.commandgroups.intake.IntakeTrackingAction;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
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
     * @param robotState     The robot state
     */
    public IntakeSampleState(String name,  IntakeSubsystem intake,
                             DropperSubsystem dropper,
                             RobotState robotState) {
        super(name, 10);
        this.robotState = robotState;
        addCommands(
                new IntakeFullReadyToTransferNoVisionAction(intake, dropper, robotState)
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
